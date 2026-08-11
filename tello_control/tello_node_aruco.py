#!/usr/bin/env python3
"""
No SIMPLES do Tello: le uma missao de um arquivo .json e executa em ordem.
Deteccao de ArUco liga em paralelo (comando "aruco", nao bloqueia a missao)
e pousa IMEDIATAMENTE assim que ve qualquer marcador, enquanto o drone
estiver voando.

Comandos do JSON:
    {"comando": "decolar"}
    {"comando": "pousar"}
    {"comando": "frente", "valor": 100}   (tambem: tras, esquerda, direita, subir, descer)
    {"comando": "girar", "valor": 90}     (negativo = anti-horario)
    {"comando": "aruco"}                  (liga deteccao - NAO bloqueia, fica ligada ate o fim)
    {"comando": "esperar", "valor": 3}

Publica no topico /tello/aruco_status (std_msgs/String) toda vez que
detecta ou deixa de detectar um marcador - abra outro terminal e rode:
    ros2 topic echo /tello/aruco_status
para acompanhar em tempo real, mesmo sem olhar pro terminal do node.
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from djitellopy import Tello
import cv2

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Correcao de espelho - ver explicacao detalhada no tello_node_json.py.
# IMAGEM_ESPELHADA = True  -> a camera esta vendo via espelho (aplica correcao)
# IMAGEM_ESPELHADA = False -> a camera ve a cena direto (nao mexe em nada)
IMAGEM_ESPELHADA = True
FLIP_MODO = 1  # 1 = horizontal, 0 = vertical, -1 = ambos

MISSAO_PATH_PADRAO = '/home/rosa/lucas_ws/tello_aruco/tello_control/missao.json'

COMANDOS_COM_VALOR = {
    'frente': (20, 500),
    'tras': (20, 500),
    'esquerda': (20, 500),
    'direita': (20, 500),
    'subir': (20, 500),
    'descer': (20, 500),
    'girar': (1, 360),
}


class TelloArucoNode(Node):
    def __init__(self):
        super().__init__('tello_aruco_node')

        self.declare_parameter('missao_path', MISSAO_PATH_PADRAO)
        missao_path = self.get_parameter('missao_path').get_parameter_value().string_value

        self.get_logger().info('Conectando ao Tello...')
        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info(f'Conectado! Bateria: {self.tello.get_battery()}%')

        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

        self.em_voo = False
        self.detectar_aruco = False
        self._pousou = False  # trava para so pousar uma vez

        # Serializa comandos ao Tello - a deteccao (thread do timer) e a
        # missao (thread propria) nao podem mandar comando ao mesmo tempo.
        self._tello_lock = threading.Lock()

        # MESMOS parametros validados no teste com a webcam.
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.adaptiveThreshWinSizeMin = 3
        aruco_params.adaptiveThreshWinSizeMax = 53
        aruco_params.adaptiveThreshWinSizeStep = 4
        aruco_params.minMarkerPerimeterRate = 0.02
        aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.status_publisher_ = self.create_publisher(String, '/tello/aruco_status', 10)

        # Deteccao roda a 10 FPS - suficiente para achar o marcador, sem
        # sobrecarregar a CPU/rede junto com o streaming de video do drone.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.missao = self.carregar_missao(missao_path)
        self.missao_thread = threading.Thread(target=self.executar_missao, daemon=True)
        self.missao_thread.start()

    def carregar_missao(self, path):
        if not os.path.isfile(path):
            self.get_logger().error(f"Arquivo de missao nao encontrado: {path}")
            return []
        with open(path, 'r', encoding='utf-8') as f:
            missao = json.load(f)
        self.get_logger().info(f"Missao carregada com {len(missao)} comandos.")
        return missao

    def publicar_status(self, texto):
        print(texto)
        msg = String()
        msg.data = texto
        self.status_publisher_.publish(msg)

    # ---------------------------------------------------------------
    # Deteccao de ArUco - roda no timer, na thread principal
    # ---------------------------------------------------------------
    def timer_callback(self):
        if not self.detectar_aruco:
            return

        frame = self.frame_read.frame
        if frame is None:
            return

        if IMAGEM_ESPELHADA:
            frame = cv2.flip(frame, FLIP_MODO)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None and self.em_voo and not self._pousou:
            ids_atuais = sorted(int(i) for i in ids.flatten())
            self._pousou = True
            self.publicar_status(f">> [ArUco] Marcador {ids_atuais} detectado - POUSANDO.")
            with self._tello_lock:
                self.tello.land()
                self.em_voo = False

    # ---------------------------------------------------------------
    # Execucao da missao - thread separada
    # ---------------------------------------------------------------
    def executar_missao(self):
        time.sleep(1.0)

        for i, item in enumerate(self.missao, start=1):
            if self._pousou:
                self.publicar_status(">> [Missao] Interrompida - ja pousou por ArUco.")
                return

            comando = item.get('comando')
            valor = item.get('valor')
            print(f"[{i}/{len(self.missao)}] {comando} {valor if valor is not None else ''}")

            try:
                if comando == 'decolar':
                    with self._tello_lock:
                        self.tello.takeoff()
                        self.em_voo = True

                elif comando == 'pousar':
                    with self._tello_lock:
                        self.tello.land()
                        self.em_voo = False

                elif comando == 'esperar':
                    time.sleep(float(valor if valor is not None else 1))

                elif comando == 'aruco':
                    self.detectar_aruco = True
                    self.publicar_status(">> [ArUco] Deteccao ativada.")

                elif comando in COMANDOS_COM_VALOR:
                    self.executar_comando_com_valor(comando, valor)

                else:
                    print(f"Comando desconhecido: {comando}")

            except Exception as e:
                self.get_logger().error(f"Erro no comando '{comando}': {e}")
                if self.em_voo and not self._pousou:
                    with self._tello_lock:
                        self.tello.land()
                        self.em_voo = False
                return

            time.sleep(1.0)

        print(">> Missao concluida.")

    def executar_comando_com_valor(self, comando, valor):
        minimo, maximo = COMANDOS_COM_VALOR[comando]
        if valor is None or not (minimo <= abs(int(valor)) <= maximo):
            raise ValueError(f"valor invalido para '{comando}': {valor}")
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
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelloArucoNode()
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