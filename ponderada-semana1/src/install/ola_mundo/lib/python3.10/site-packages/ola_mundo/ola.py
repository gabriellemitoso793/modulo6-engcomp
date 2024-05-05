import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
import time
import math

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def move_turtle(self, linear, angular):
        # Envia comandos de movimento para a tartaruga
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
        time.sleep(1)  # Pausa de 1 segundo entre os movimentos

def draw_heart(controller):
    # Movimentos para desenhar um coração
    movements = [
        (0.0, 0.0),                      # Iniciar (sem movimento)
        (2.0, 0.0),                      # Avançar para a primeira parte do coração
        (-2.0, 0.0),                     # Voltar ao centro (mover para trás)
        (0.0, math.radians(135)),       # Virar para a direita (parte superior do coração)
        (math.sqrt(2), math.sqrt(2)),    # Avançar em diagonal (parte superior do coração)
        (0.0, -math.radians(135)),      # Virar para a esquerda (parte superior do coração)
        (-math.sqrt(2), math.sqrt(2)),   # Voltar em diagonal (parte superior do coração)
        (0.0, math.radians(90)),        # Virar para a direita (parte inferior do coração)
        (1.0, 0.0),                     # Avançar para a base do coração
        (0.0, 0.0)                      # Parar
    ]

    # Executa os movimentos para desenhar o coração
    for linear, angular in movements:
        controller.move_turtle(linear, angular)

def set_pen_color(controller, r, g, b):
    # Configura a cor da caneta da tartaruga
    set_pen_client = controller.create_client(SetPen, '/turtle1/set_pen')

    while not set_pen_client.wait_for_service(timeout_sec=1.0):
        print('Serviço SetPen não disponível, aguardando...')

    request = SetPen.Request()
    request.r = r
    request.g = g
    request.b = b
    request.width = 5  # Largura da linha
    request.off = 0    # Caneta ligada

    future = set_pen_client.call_async(request)
    rclpy.spin_until_future_complete(controller, future)

    if future.result() is not None:
        print('Cor da caneta configurada com sucesso!')
    else:
        print('Falha ao configurar a cor da caneta.')

def spawn_turtle(controller):
    # Cria uma nova tartaruga no Turtlesim
    spawn_client = controller.create_client(Spawn, '/spawn')

    while not spawn_client.wait_for_service(timeout_sec=1.0):
        print('Serviço de spawn não disponível, aguardando...')

    request = Spawn.Request()
    request.x = 5.0
    request.y = 5.0
    request.theta = 0.0
    request.name = 'my_turtle'

    future = spawn_client.call_async(request)
    rclpy.spin_until_future_complete(controller, future)

    if future.result() is not None:
        print('Nova tartaruga criada com sucesso:', future.result().name)
    else:
        print('Falha ao criar uma nova tartaruga.')

def kill_turtle(controller):
    # Mata a tartaruga criada anteriormente
    kill_client = controller.create_client(Kill, '/kill')

    while not kill_client.wait_for_service(timeout_sec=1.0):
        print('Serviço de kill não disponível, aguardando...')

    request = Kill.Request()
    request.name = 'my_turtle'

    future = kill_client.call_async(request)
    rclpy.spin_until_future_complete(controller, future)

    if future.result() is not None:
        print('Tartaruga morta com sucesso.')
    else:
        print('Falha ao matar a tartaruga.')

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()

    print("Desenhando um coração no Turtlesim...")

    # Configura a cor da caneta para rosa
    set_pen_color(controller, 255, 192, 203)

    # Desenha o coração
    draw_heart(controller)

    # Aguarda a conclusão dos movimentos do coração
    time.sleep(10)  # Ajuste o tempo conforme necessário para o coração ser desenhado completamente

    # Cria uma nova tartaruga no Turtlesim
    spawn_turtle(controller)

    # Aguarda um pouco antes de matar a tartaruga
    time.sleep(2)

    # Mata a tartaruga recém-criada no Turtlesim
    kill_turtle(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

