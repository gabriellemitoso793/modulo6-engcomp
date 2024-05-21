import typer  # Biblioteca para criar interfaces de linha de comando
import rclpy  # Biblioteca ROS 2 para Python
from rclpy.node import Node  # Classe base para criar um nó ROS 2
from geometry_msgs.msg import Twist  # Mensagem para controle de movimento
from std_srvs.srv import Empty  # Serviço ROS 2 vazio, usado para parada de emergência
import inquirer  # Biblioteca para criar interfaces em linha de comando
import threading  # Biblioteca para trabalhar com threads
import sys  # Biblioteca padrão do Python para manipulação do sistema
import os  # Biblioteca padrão do Python para operações do sistema operacional
import time  # Biblioteca padrão do Python para manipulação de tempo

# Importações específicas para lidar com entrada de teclado dependendo do sistema operacional
if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

app = typer.Typer()  # Inicialização do aplicativo Typer

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')  # Inicializa a classe base Node
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # Cria um publicador para o tópico 'cmd_vel'
        self.emergency_client = self.create_client(Empty, 'emergency_stop')  # Cria um cliente para o serviço de parada de emergência
        self.connected = False  # Estado de conexão inicial do robô
        self.linear_speed = 0.0  # Velocidade linear inicial
        self.angular_speed = 0.0  # Velocidade angular inicial

    def print_status(self):
        # Printa o status atual do robô
        status = "Conectado" if self.connected else "Desconectado"
        print(f"Status: {status}, Velocidade Linear: {self.linear_speed:.2f} m/s, Velocidade Angular: {self.angular_speed:.2f} rad/s")

    def connect(self):
        # Conecta o robô e printa o status
        self.connected = True
        print("Robô conectado e pronto para publicar.")
        self.print_status()

    def disconnect(self):
        # Desconecta o robô e printa o status
        self.connected = False
        print("Robô desconectado, para utilizar conecte-o novamente.")
        self.print_status()

    def move_robot(self):
        # Publica uma mensagem de movimento se o robô estiver conectado
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
        self.print_status()

    def stop_robot(self):
        # Para o movimento do robô
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Parando robô.")
        self.print_status()

    def adjust_speed(self, linear_change=0.0, angular_change=0.0):
        # Ajusta a velocidade do robô e publica a nova velocidade
        self.linear_speed += linear_change
        self.angular_speed += angular_change
        self.move_robot()

    def send_emergency_stop(self):
        # Envia um sinal de parada de emergência
        if not self.emergency_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            return
        req = Empty.Request()
        self.future = self.emergency_client.call_async(req)
        self.future.add_done_callback(self.emergency_stop_callback)

    def emergency_stop_callback(self, future):
        # Callback para lidar com a resposta do serviço de parada de emergência
        try:
            future.result()
            self.get_logger().info('Emergency stop signal sent successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to call emergency stop service: {e}')

    def kill_switch(self):
        # Força a parada do robô e do processo ROS
        print("Forçando a parada do processo.")
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        self.send_emergency_stop()
        rclpy.shutdown()
        main()  # Reinicia o programa

def get_key(settings):
    # Captura a entrada do teclado dependendo do sistema operacional
    if os.name == 'nt':
        if msvcrt.kbhit():
            return msvcrt.getch().decode()
        return ''
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def show_controls():
    # Mostra os controles para o usuário
    print("Entrando no modo de teleoperação. Use as seguintes teclas para controlar o robô:")
    print("  _______ ")
    print(" |   w   |")
    print(" | a s d |")
    print(" |_______|")
    print(" Use 'w', 's', 'a', 'd' para mover.")
    print(" Use 'espaço' para parar.")
    print(" Use 'q' para sair.")
    print(" Use 'b' kill switch.")

def teleop_mode(robot_controller):
    # Modo de teleoperação para controlar o robô usando o teclado
    settings = termios.tcgetattr(sys.stdin) if os.name != 'nt' else None
    show_controls()

    try:
        while True:
            key = get_key(settings)
            actions = {
                'w': lambda: robot_controller.adjust_speed(linear_change=0.1),
                's': lambda: robot_controller.adjust_speed(linear_change=-0.1),
                'a': lambda: robot_controller.adjust_speed(angular_change=0.1),
                'd': lambda: robot_controller.adjust_speed(angular_change=-0.1),
                ' ': robot_controller.stop_robot,
                'b': robot_controller.kill_switch,
                'q': lambda: "break"
            }
            result = actions.get(key, lambda: None)()
            if result == "break":
                break
            time.sleep(0.1)
    finally:
        if os.name != 'nt' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def user_interaction(robot_controller):
    # Interação com o usuário através do menu de escolhas CLI
    while True:
        questions = [
            inquirer.List('action',
                          message="Qual ação você quer realizar? (Para teleoperar, deve-se conectar antes)",
                          choices=['Teleoperar', 'Conectar', 'Desconectar', "Parada de emergência", 'Sair'])
        ]
        actions = {
            'Teleoperar': lambda: teleop_mode(robot_controller),
            'Conectar': robot_controller.connect,
            'Desconectar': robot_controller.disconnect,
            'Parada de emergência': robot_controller.kill_switch,
            'Sair': lambda: "exit"
        }
        answers = inquirer.prompt(questions)
        action = answers['action']
        result = actions[action]()
        if result == "exit":
            break
    robot_controller.disconnect()
    rclpy.shutdown()

@app.command()
def main():
    # Função principal que inicializa o ROS, cria o controlador do robô e gerencia threads
    rclpy.init()
    robot_controller = RobotController()
    monitor_thread = threading.Thread(target=robot_controller.print_status)
    monitor_thread.start()
    user_thread = threading.Thread(target=user_interaction, args=(robot_controller,))
    user_thread.start()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.disconnect()
        rclpy.shutdown()
    finally:
        user_thread.join()
        monitor_thread.join()

if __name__ == '__main__':
    main()  # Executa a função principal se o script for executado diretamente

