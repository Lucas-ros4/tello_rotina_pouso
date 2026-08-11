#!/usr/bin/env python3
"""
No Mestre do Tello: Controle por Waypoints + Stream de Camera + Publicador ROS 2
+ Deteccao de marcadores ArUco.

Comandos do menu:
  add <comando> <valor>   -> adiciona um waypoint na missao (ex: add girar 90 / add frente 100)
  add decolar / add pousar -> adiciona decolagem ou pouso
  listar                   -> mostra a missao atual
  remover <indice>         -> remove um waypoint (indice comeca em 1)
  limpar                   -> apaga toda a missao
  executar                 -> executa a missao do inicio ao fim
  camera                   -> liga / desliga a janela de visualizacao da camera
  aruco                    -> liga / desliga a deteccao de marcadores ArUco (mostra a janela junto)
  bateria                  -> mostra a bateria
  sair                     -> encerra (pousa se estiver voando)
"""

import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2

COMANDOS_COM_VALOR = {
    'frente': (20, 500),
    'tras': (20, 500),
    'esquerda': (20, 500),
    'direita': (20, 500),
    'subir': (20, 500),
    'descer': (20, 500),
    'girar': (1, 360),
}
COMANDOS_SEM_VALOR = {'decolar', 'pousar'}

# ----------------------------------------------------------------------
# Config do ArUco - mesmo dicionario usado no gerar_marcadores.py
# ----------------------------------------------------------------------
ARUCO_DICT = cv2.aruco.DICT_4X4_50


class TelloWaypointNode(Node):
    def __init__(self):
        super().__init__('tello_waypoint_node')

        self.get_logger().info('Conectando ao Tello...')
        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info(f'Conectado! Bateria: {self.tello.get_battery()}%')

        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

        self.em_voo = False
        self.missao = []

        self.bridge = CvBridge()
        self.window_name = "Tello_Camera_Waypoints"

        self._camera_lock = threading.Lock()
        self.mostrar_camera = False
        self._janela_aberta = False

        # --- Estado do modo ArUco ---
        self._aruco_lock = threading.Lock()
        self.detectar_aruco = False
        self._ultimo_ids_detectados = None

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.publisher_ = self.create_publisher(Image, '/tello/image_raw', 10)

        self.timer = self.create_timer(0.033, self.timer_callback)

        self.menu_thread = threading.Thread(target=self.rodar_menu, daemon=True)
        self.menu_thread.start()

    def timer_callback(self):
        frame = self.frame_read.frame
        if frame is not None:
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
                print(">> Janela da camera LIGADA. "
                      "OBS: clique de volta no terminal para continuar digitando comandos.")
            elif not quer_mostrar and self._janela_aberta:
                cv2.destroyWindow(self.window_name)
                self._janela_aberta = False
                print(">> Janela da camera DESLIGADA.")

            if self._janela_aberta:
                cv2.imshow(self.window_name, frame_para_mostrar)
                cv2.waitKey(1)

    def processar_aruco(self, frame):
        """So identifica os marcadores - nao decide nada sobre pouso/movimento."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        frame_desenhado = frame.copy()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame_desenhado, corners, ids)
            ids_atuais = sorted(int(i) for i in ids.flatten())

            if ids_atuais != self._ultimo_ids_detectados:
                print(f">> [ArUco] Marcador(es) detectado(s): {ids_atuais}")
                self._ultimo_ids_detectados = ids_atuais
        else:
            if self._ultimo_ids_detectados is not None:
                print(">> [ArUco] Nenhum marcador visivel no momento.")
                self._ultimo_ids_detectados = None

        return frame_desenhado

    def alternar_camera(self):
        with self._camera_lock:
            self.mostrar_camera = not self.mostrar_camera

    def alternar_aruco(self):
        with self._aruco_lock:
            self.detectar_aruco = not self.detectar_aruco
            ligado = self.detectar_aruco
        estado = "LIGADA" if ligado else "DESLIGADA"
        print(f">> Deteccao de ArUco {estado}.")

    def rodar_menu(self):
        print("\n" + "=" * 55)
        print("        MISSAO POR WAYPOINTS - TELLO")
        print("=" * 55)
        print(__doc__)
        print("=" * 55)

        while rclpy.ok():
            try:
                entrada = input("\n[missao] > ").strip().lower().split()
                if not entrada:
                    continue

                cmd = entrada[0]

                if cmd == 'sair':
                    if self.em_voo:
                        print("Pousando por seguranca antes de sair...")
                        self.tello.land()
                    with self._camera_lock:
                        self.mostrar_camera = False
                    with self._aruco_lock:
                        self.detectar_aruco = False
                    rclpy.shutdown()
                    break

                elif cmd == 'bateria':
                    print(f"Bateria atual: {self.tello.get_battery()}%")

                elif cmd == 'camera':
                    self.alternar_camera()

                elif cmd == 'aruco':
                    self.alternar_aruco()

                elif cmd == 'add':
                    self.adicionar_waypoint(entrada[1:])

                elif cmd == 'listar':
                    self.listar_missao()

                elif cmd == 'remover':
                    self.remover_waypoint(entrada[1:])

                elif cmd == 'limpar':
                    self.missao = []
                    print(">> Missao limpa.")

                elif cmd == 'executar':
                    self.executar_missao()

                else:
                    print("Comando desconhecido. Digite 'add', 'listar', 'remover', "
                          "'limpar', 'executar', 'camera', 'aruco', 'bateria' ou 'sair'.")

            except KeyboardInterrupt:
                print("\nForcando pouso de emergencia...")
                if self.em_voo:
                    self.tello.land()
                with self._camera_lock:
                    self.mostrar_camera = False
                with self._aruco_lock:
                    self.detectar_aruco = False
                rclpy.shutdown()
                break

    def adicionar_waypoint(self, args):
        if not args:
            print("Erro: falta o comando. Ex: add girar 90")
            return

        comando = args[0]

        if comando in COMANDOS_SEM_VALOR:
            self.missao.append((comando, None))
            print(f">> Waypoint adicionado: {comando}")
            return

        if comando not in COMANDOS_COM_VALOR:
            print(f"Erro: comando '{comando}' invalido.")
            return

        if len(args) < 2:
            print(f"Erro: falta o valor. Ex: add {comando} 50")
            return

        try:
            valor = int(args[1])
        except ValueError:
            print("Erro: o valor deve ser um numero inteiro.")
            return

        minimo, maximo = COMANDOS_COM_VALOR[comando]
        if not (minimo <= abs(valor) <= maximo):
            print(f"Erro: valor fora do intervalo permitido ({minimo} a {maximo}).")
            return

        self.missao.append((comando, valor))
        print(f">> Waypoint adicionado: {comando} {valor}")

    def listar_missao(self):
        if not self.missao:
            print("Missao vazia.")
            return
        print(f"\nMissao atual ({len(self.missao)} waypoints):")
        for i, (cmd, valor) in enumerate(self.missao, start=1):
            valor_str = f" {valor}" if valor is not None else ""
            print(f"  {i}. {cmd}{valor_str}")

    def remover_waypoint(self, args):
        if not args:
            print("Erro: informe o indice. Ex: remover 2")
            return
        try:
            idx = int(args[0]) - 1
        except ValueError:
            print("Erro: indice invalido.")
            return
        if idx < 0 or idx >= len(self.missao):
            print("Erro: indice fora do range da missao.")
            return
        removido = self.missao.pop(idx)
        print(f">> Removido: {removido[0]} {removido[1] or ''}")

    def executar_missao(self):
        if not self.missao:
            print("Missao vazia, nada a executar.")
            return

        print(f"\nExecutando missao com {len(self.missao)} waypoints...\n")
        for i, (cmd, valor) in enumerate(self.missao, start=1):
            print(f"[{i}/{len(self.missao)}] {cmd} {valor if valor is not None else ''}")
            try:
                self.executar_comando(cmd, valor)
            except Exception as e:
                self.get_logger().error(f"Erro no waypoint {i} ({cmd}): {e}")
                print("Abortando missao por seguranca.")
                if self.em_voo:
                    self.tello.land()
                    self.em_voo = False
                return
            time.sleep(1.0)

        print("\n>> Missao concluida com sucesso.")

    def executar_comando(self, cmd, valor):
        if cmd == 'decolar':
            self.tello.takeoff()
            self.em_voo = True
        elif cmd == 'pousar':
            self.tello.land()
            self.em_voo = False
        elif cmd == 'frente':
            self.tello.move_forward(valor)
        elif cmd == 'tras':
            self.tello.move_back(valor)
        elif cmd == 'esquerda':
            self.tello.move_left(valor)
        elif cmd == 'direita':
            self.tello.move_right(valor)
        elif cmd == 'subir':
            self.tello.move_up(valor)
        elif cmd == 'descer':
            self.tello.move_down(valor)
        elif cmd == 'girar':
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
        node = TelloWaypointNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Encerrado: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
