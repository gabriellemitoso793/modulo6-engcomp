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
        self.create_client(Spawn, '/spawn')
        self.create_client(Kill, '/kill')
        self.create_client(SetPen, '/turtle1/set_pen')

    def move_turtle(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

    def spawn_turtle(self):
        # Solicitar o serviço de spawn para criar uma nova tartaruga
        spawn_client = self.create_client(Spawn, '/spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de spawn não está disponível. Tentando novamente...')
        
        request = Spawn.Request()
        request.x = 5.0
        request.y = 5.0
        request.theta = 0.0
        request.name = 'my_turtle'

        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Tartaruga spawnada com sucesso: %s', future.result().name)
        else:
            self.get_logger().info('Falha ao spawnar a tartaruga')

    def kill_turtle(self):
        # Solicitar o serviço de kill para remover a tartaruga
        kill_client = self.create_client(Kill, '/kill')
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de kill não está disponível. Tentando novamente...')
        
        request = Kill.Request()
        request.name = 'my_turtle'

        future = kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Tartaruga morta com sucesso')
        else:
            self.get_logger().info('Falha ao matar a tartaruga')

def draw_pink_star(controller):
    # Movimentos para desenhar uma estrela
    movements = [
        (0.0, 0.0),      # Iniciar
        (1.0, 0.0),      # Primeira ponta
        (-1.0, 0.0),     # Voltar ao centro
        (0.0, 0.0),      # Parar
        (0.0, 1.0),      # Segunda ponta
        (0.0, -1.0),     # Voltar ao centro
        (0.0, 0.0),      # Parar
        (1.0, 1.0),      # Terceira ponta
        (-1.0, -1.0),    # Voltar ao centro
        (0.0, 0.0),      # Parar
        (-1.0, 0.0),     # Quarta ponta
        (1.0, 0.0),      # Voltar ao centro
        (0.0, 0.0)       # Parar
    ]

    for linear, angular in movements:
        controller.move_turtle(linear, angular)
        time.sleep(1) 

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()

    print("Desenhando uma estrela no Turtlesim...")
    
    # Configurar a cor da caneta para rosa (vermelho + azul)
    set_pen_color(controller, 255, 0, 255)
    
    # Spawn da tartaruga
    controller.spawn_turtle()
    
    # Desenhar a estrela rosa
    draw_pink_star(controller)
    
    # Pausa para visualizar o desenho
    time.sleep(2)
    
    # Kill da tartaruga
    controller.kill_turtle()
    
    # Encerrar a execução
    controller.destroy_node()
    rclpy.shutdown()

def set_pen_color(controller, r, g, b):
    # Configurar a cor da caneta
    set_pen_client = controller.create_client(SetPen, '/turtle1/set_pen')

    while not set_pen_client.wait_for_service(timeout_sec=1.0):
        controller.get_logger().info('Serviço SetPen não está disponível. Tentando novamente...')
    
    request = SetPen.Request()
    request.r = r
    request.g = g
    request.b = b
    request.width = 5  
    request.off = 0    
    
    future = set_pen_client.call_async(request)
    rclpy.spin_until_future_complete(controller, future)

    if future.result() is not None:
        controller.get_logger().info('Cor da caneta configurada com sucesso')
    else:
        controller.get_logger().info('Falha ao configurar a cor da caneta')

if __name__ == '__main__':
    main()
