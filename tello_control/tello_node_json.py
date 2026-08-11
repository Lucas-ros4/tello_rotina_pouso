#!/usr/bin/env python3
"""
Comandos para colocar no arquivo json
    {"comando": "decolar"}
    {"comando": "pousar"}
    {"comando": "frente", "valor": 100}      (tras, esquerda, direita, subir, descer)
    {"comando": "girar", "valor": 90}        (negativo = anti-horario)
    {"comando": "aruco"}                     (ativa deteccao continua - NAO bloqueia a missao())
    {"comando": "camera"}                    (liga a janela de visualizacao da camera)
    {"comando": "esperar", "valor": 3}

    - "aruco" e NAO-BLOQUEANTE: liga a deteccao e a missao segue na mesma hora.
    - So reage a aruco com os ID selecionados em IDS_VALIDOS_POUSO
    - Ao ver um ID valido a missao e cancelada e vai para a rotina de pouso
      e comeca a tentar se alinhar sobre o marcador (lateral, depois
      frente/tras, depois desce), em passos de PASSO_CM.
    - tem um tempo para se ele não conseguir alinhar dentro do tempo ele pousa de onde ele estiver (isso foi implementado devido ao tello ter o ajusto minimo de 20 cm e tavez ficar preso infinitamente na rotina de pouso e nunca pousar)

"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Correcao de espelho (ex: camera vendo atraves de um espelho fisico a 45
# graus). True = aplica a correcao. False = imagem normal, nao mexe em nada.
IMAGEM_ESPELHADA = True
FLIP_MODO = 1  # 1 = horizontal, 0 = vertical, -1 = ambos

# arametros do pouso alinhado
TEMPO_LIMITE_POUSO_S = 45.0   # prazo maximo tentando alinhar antes de pousar onde estiver
TOLERANCIA_PX = 40             # abaixo disso, considera o eixo "centralizado"
PASSO_CM = 20                  # passo minimo de movimento do Tello 
ALTURA_POUSO_CM = 40           # abaixo disso, se alinhado, pousa em vez de continuar descendo
FRAMES_SEM_MARCADOR_MAX = 40   # tolerancia de frames sem ver o marcador antes de desistir
INVERTER_FRENTE_TRAS = True    # a adaptacao de camera inverte frente/tras
IDS_VALIDOS_POUSO = set(range(1, 6))  # so pousa se detectar um desses IDs: {1,2,3,4,5}

COMANDOS_COM_VALOR = {
    'frente': (20, 500),
    'tras': (20, 500),
    'esquerda': (20, 500),
    'direita': (20, 500),
    'subir': (20, 500),
    'descer': (20, 500),
    'girar': (1, 360),
}


class TelloJsonNode(Node):
    def __init__(self):
        super().__init__('tello_json_node')

        MISSAO_PATH_PADRAO = '/home/rosa/lucas_ws/tello_aruco/tello_control/missao.json'
        self.declare_parameter('missao_path', MISSAO_PATH_PADRAO)
        missao_path = self.get_parameter('missao_path').get_parameter_value().string_value

        self.get_logger().info('Conectando ao Tello...')
        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info(f'Conectado! Bateria: {self.tello.get_battery()}%')

        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

        self.em_voo = False
        self.bridge = CvBridge()
        self.window_name = "Tello_Camera"

        self._camera_lock = threading.Lock()
        self.mostrar_camera = False
        self._janela_aberta = False

        self._aruco_lock = threading.Lock()
        self.detectar_aruco = False
        self._ultimo_status_ts = 0.0

        # Serializa todo comando ao Tello (o drone so aceita um por vez).
        self._tello_lock = threading.Lock()
        # Ligado assim que um ID valido e visto pela 1a vez - a missao para
        # de mandar novos comandos a partir dai.
        self._pouso_solicitado = threading.Event()

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.adaptiveThreshWinSizeMin = 3
        aruco_params.adaptiveThreshWinSizeMax = 53#limite de hardware)
        aruco_params.adaptiveThreshWinSizeStep = 4
        aruco_params.minMarkerPerimeterRate = 0.02
        aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.publisher_ = self.create_publisher(Image, '/tello/image_raw', 10)
        self.status_publisher_ = self.create_publisher(String, '/tello/status', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.missao = self.carregar_missao(missao_path)
        self.missao_thread = threading.Thread(target=self.executar_missao, daemon=True)
        self.missao_thread.start()

    def publicar_status(self, texto):
        """Publica no /tello/status e imprime no terminal - uma unica fonte
        pros dois ficarem sempre sincronizados."""
        print(texto)
        msg = String()
        msg.data = texto
        self.status_publisher_.publish(msg)

    def carregar_missao(self, path):
        if not os.path.isfile(path):
            self.get_logger().error(f"Arquivo de missao nao encontrado: {path}")
            return []
        with open(path, 'r', encoding='utf-8') as f:
            missao = json.load(f)
        self.get_logger().info(f"Missao carregada com {len(missao)} comandos.")
        return missao

    # Camera / ArUco - roda a 30 FPS
    def timer_callback(self):
        frame = self.frame_read.frame
        if frame is None:
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        except Exception:
            pass

        with self._aruco_lock:
            quer_detectar = self.detectar_aruco

        frame_para_mostrar = frame
        if quer_detectar:
            frame_para_mostrar = self.processar_aruco(frame)

        with self._camera_lock:
            quer_mostrar = self.mostrar_camera or quer_detectar

        if quer_mostrar and not self._janela_aberta:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 480, 360)
            cv2.moveWindow(self.window_name, 50, 50)
            self._janela_aberta = True
        elif not quer_mostrar and self._janela_aberta:
            cv2.destroyWindow(self.window_name)
            self._janela_aberta = False

        if self._janela_aberta:
            cv2.imshow(self.window_name, frame_para_mostrar)
            cv2.waitKey(1)

    def processar_aruco(self, frame):
        """Detecta a cada frame. Ao ver um ID valido pela 1a vez (com o
        drone voando), dispara a rotina de pouso alinhado numa thread
        separada, para nao travar a publicacao de imagem/GUI."""
        try:
            if IMAGEM_ESPELHADA:
                frame = cv2.flip(frame, FLIP_MODO)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        except Exception as e:
            self.get_logger().error(f"Erro na deteccao de ArUco: {e}")
            return frame

        frame_desenhado = frame.copy()

        if ids is None:
            return frame_desenhado

        cv2.aruco.drawDetectedMarkers(frame_desenhado, corners, ids)
        ids_atuais = sorted(int(i) for i in ids.flatten())
        ids_validos = [i for i in ids_atuais if i in IDS_VALIDOS_POUSO]

        agora = time.monotonic()
        if ids_validos and agora - self._ultimo_status_ts >= 1.0:
            self.publicar_status(f">> ArUco {ids_validos} identificado.")
            self._ultimo_status_ts = agora

        if self.em_voo and ids_validos and not self._pouso_solicitado.is_set():
            self.publicar_status(
                f">> DECISAO: marcador {ids_validos} valido - cancelando missao "
                f"e pousando alinhado (limite {TEMPO_LIMITE_POUSO_S:.0f}s)."
            )
            self._pouso_solicitado.set()
            threading.Thread(target=self.rotina_pouso_alinhado, daemon=True).start()

        return frame_desenhado

    def _mover_frente(self, cm):
        if INVERTER_FRENTE_TRAS:
            self.tello.move_back(cm)
        else:
            self.tello.move_forward(cm)

    def _mover_tras(self, cm):
        if INVERTER_FRENTE_TRAS:
            self.tello.move_forward(cm)
        else:
            self.tello.move_back(cm)

    def rotina_pouso_alinhado(self):
        """Centraliza sobre o marcador (lateral, depois frente/tras) e
        desce em passos, ate pousar. Pousa onde estiver se o prazo estourar
        ou se perder o marcador de vista por tempo demais."""
        self.publicar_status(">> [Pouso] Alinhando sobre o marcador...")
        prazo_final = time.monotonic() + TEMPO_LIMITE_POUSO_S
        frames_sem_marcador = 0
        desfazer_ultimo_movimento = None  # (funcao, cm) para reverter se perder o marcador

        try:
            while rclpy.ok() and self.em_voo:
                if time.monotonic() >= prazo_final:
                    self.publicar_status(">> [Pouso] Tempo esgotado - pousando onde esta.")
                    break

                frame = self.frame_read.frame
                if frame is None:
                    time.sleep(0.05)
                    continue

                if IMAGEM_ESPELHADA:
                    frame = cv2.flip(frame, FLIP_MODO)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.aruco_detector.detectMarkers(gray)

                ids_flat = ids.flatten() if ids is not None else []
                indice_valido = next(
                    (idx for idx, mid in enumerate(ids_flat) if int(mid) in IDS_VALIDOS_POUSO),
                    None
                )

                if indice_valido is None:
                    frames_sem_marcador += 1

                    if frames_sem_marcador == 1 and desfazer_ultimo_movimento is not None:
                        func, cm = desfazer_ultimo_movimento
                        self.publicar_status(">> [Pouso] Marcador sumiu - desfazendo ultimo movimento.")
                        with self._tello_lock:
                            try:
                                func(cm)
                            except Exception as e:
                                self.get_logger().warn(f"Falha ao desfazer movimento: {e}")
                        desfazer_ultimo_movimento = None
                        time.sleep(0.6)
                        continue

                    if frames_sem_marcador > FRAMES_SEM_MARCADOR_MAX:
                        self.publicar_status(">> [Pouso] Marcador perdido de vista - pousando onde esta.")
                        break
                    time.sleep(0.1)
                    continue

                frames_sem_marcador = 0
                desfazer_ultimo_movimento = None

                c = corners[indice_valido][0]
                centro_marcador = c.mean(axis=0)
                altura_img, largura_img = frame.shape[:2]
                dx = centro_marcador[0] - largura_img / 2.0
                dy = centro_marcador[1] - altura_img / 2.0
                restante_s = prazo_final - time.monotonic()

                self.publicar_status(
                    f">> [Pouso] marcador {int(ids_flat[indice_valido])} "
                    f"dx={dx:+.0f}px dy={dy:+.0f}px (restam {restante_s:.0f}s)"
                )

                with self._tello_lock:
                    try:
                        if abs(dx) > TOLERANCIA_PX:
                            if dx > 0:
                                self.tello.move_right(PASSO_CM)
                                desfazer_ultimo_movimento = (self.tello.move_left, PASSO_CM)
                            else:
                                self.tello.move_left(PASSO_CM)
                                desfazer_ultimo_movimento = (self.tello.move_right, PASSO_CM)
                        elif abs(dy) > TOLERANCIA_PX:
                            if dy > 0:
                                self._mover_frente(PASSO_CM)
                                desfazer_ultimo_movimento = (self._mover_tras, PASSO_CM)
                            else:
                                self._mover_tras(PASSO_CM)
                                desfazer_ultimo_movimento = (self._mover_frente, PASSO_CM)
                        else:
                            altura_atual = self.tello.get_height()
                            if altura_atual <= ALTURA_POUSO_CM:
                                self.publicar_status(">> [Pouso] Alinhado e baixo o suficiente - pousando!")
                                break
                            self.tello.move_down(PASSO_CM)
                            desfazer_ultimo_movimento = (self.tello.move_up, PASSO_CM)
                    except Exception as e:
                        self.get_logger().warn(f"Comando de ajuste falhou: {e}")

                time.sleep(0.6)

        except Exception as e:
            self.get_logger().error(f"Erro na rotina de pouso alinhado: {e}")

        finally:
            with self._tello_lock:
                try:
                    self.tello.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.2)
                except Exception:
                    pass
                try:
                    self.tello.land()
                except Exception as e:
                    self.get_logger().error(f"Erro ao pousar: {e}")
                self.em_voo = False
            self.publicar_status(">> [Pouso] Concluido.")

    # ---------------------------------------------------------------
    # Execucao da missao
    # ---------------------------------------------------------------
    def executar_missao(self):
        time.sleep(1.0)

        for i, item in enumerate(self.missao, start=1):
            if self._pouso_solicitado.is_set():
                self.publicar_status(">> Missao interrompida - pouso por ArUco em andamento.")
                return

            comando = item.get('comando')
            valor = item.get('valor')
            self.publicar_status(f"[{i}/{len(self.missao)}] {comando}" + (f" {valor}" if valor is not None else ""))

            try:
                if comando == 'decolar':
                    with self._tello_lock:
                        self.tello.takeoff()
                        self.em_voo = True

                elif comando == 'pousar':
                    with self._tello_lock:
                        self.tello.land()
                        self.em_voo = False

                elif comando == 'camera':
                    with self._camera_lock:
                        self.mostrar_camera = True

                elif comando == 'esperar':
                    time.sleep(float(valor if valor is not None else 1))

                elif comando == 'aruco':
                    with self._aruco_lock:
                        self.detectar_aruco = True
                    self.publicar_status(">> Deteccao de ArUco ativada.")

                elif comando in COMANDOS_COM_VALOR:
                    self.executar_comando_com_valor(comando, valor)

                else:
                    self.publicar_status(f">> Comando desconhecido: '{comando}', ignorando.")

            except Exception as e:
                self.get_logger().error(f"Erro no comando {i} ({comando}): {e}")
                self.publicar_status(">> Abortando missao por seguranca.")
                if self.em_voo and not self._pouso_solicitado.is_set():
                    with self._tello_lock:
                        self.tello.land()
                        self.em_voo = False
                return

            time.sleep(1.0)

        self.publicar_status(">> Missao concluida. Deteccao de ArUco continua ativa.")

    def executar_comando_com_valor(self, comando, valor):
        minimo, maximo = COMANDOS_COM_VALOR[comando]
        if valor is None or not (minimo <= abs(int(valor)) <= maximo):
            raise ValueError(f"valor invalido para '{comando}': {valor} (esperado {minimo}-{maximo})")
        valor = int(valor)

        with self._tello_lock:
            if comando == 'frente':
                self.tello.move_forward(valor)
            elif comando == 'tras':
                self.tello.move_back(valor)
            elif comando == 'esquerda':
                self.tello.move_left(valor)
            elif comando == 'direita':
                self.tello.move_right(valor)
            elif comando == 'subir':
                self.tello.move_up(valor)
            elif comando == 'descer':
                self.tello.move_down(valor)
            elif comando == 'girar':
                if valor >= 0:
                    self.tello.rotate_clockwise(valor)
                else:
                    self.tello.rotate_counter_clockwise(abs(valor))

    def destroy_node(self):
        try:
            self.tello.streamoff()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelloJsonNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Encerrado: {e}")
    finally:
        if node is not None:
            if node.em_voo:
                try:
                    node.tello.land()
                except Exception:
                    pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()