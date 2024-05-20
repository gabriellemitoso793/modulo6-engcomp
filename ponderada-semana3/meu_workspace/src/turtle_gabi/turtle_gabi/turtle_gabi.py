import typer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import inquirer
import threading
import sys
import os
import time

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

app = typer.Typer()

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_client = self.create_client(Empty, 'emergency_stop')
        self.connected = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def print_status(self):
        status = "Conectado" if self.connected else "Desconectado"
        print(f"Status: {status}, Velocidade Linear: {self.linear_speed:.2f} m/s, Velocidade Angular: {self.angular_speed:.2f} rad/s")

    def connect(self):
        self.connected = True
        print("Robô conectado e pronto para publicar.")
        self.print_status()

    def disconnect(self):
        self.connected = False
        print("Robô desconectado, para utilizar conecte-o novamente.")
        self.print_status()

    def move_robot(self):
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
        self.print_status()

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Parando robô.")
        self.print_status()

    def adjust_speed(self, linear_change=0.0, angular_change=0.0):
        self.linear_speed += linear_change
        self.angular_speed += angular_change
        self.move_robot()

    def send_emergency_stop(self):
        if not self.emergency_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            return
        req = Empty.Request()
        self.future = self.emergency_client.call_async(req)
        self.future.add_done_callback(self.emergency_stop_callback)

    def emergency_stop_callback(self, future):
        try:
            future.result()
            self.get_logger().info('Emergency stop signal sent successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to call emergency stop service: {e}')

    def kill_switch(self):
        print("Forçando a parada do processo.")
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        self.send_emergency_stop()
        rclpy.shutdown()
        main()

def get_key(settings):
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
    main()
