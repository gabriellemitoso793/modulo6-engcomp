import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import streamlit as st
import threading
import time
import cv2
import base64
import numpy as np
from PIL import Image
import io
import queue
from streamlit.runtime.scriptrunner import add_script_run_ctx, get_script_run_ctx

# Cria uma fila para gerenciar atualizações da interface do usuário
ui_queue = queue.Queue(maxsize=10) 

# Classe que controla o robô
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Cria um publisher para enviar comandos de velocidade para o robô
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile=qos_profile_sensor_data
        )
        # Inicializa as variáveis de velocidade linear e angular
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.killed = False  # Flag para o estado de desligamento do robô
        self.safety_distance = 0.35  # Distância de segurança para parar o robô
        self.front_clear = True  # Flag para indicar se a frente está livre
        self.back_clear = True  # Flag para indicar se a traseira está livre

    # Callback para processar dados do LIDAR
    def lidar_callback(self, msg):
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 12
        # Define os setores para a frente e para trás
        front_left_indices = range(num_ranges - sector_size, num_ranges)
        front_right_indices = range(0, sector_size)
        back_indices = range(5 * sector_size, 7 * sector_size)
        # Filtra as leituras do LIDAR para os setores definidos
        front_ranges = [msg.ranges[i] for i in front_left_indices if 0.01 < msg.ranges[i] < 100.0] + \
                       [msg.ranges[i] for i in front_right_indices if 0.01 < msg.ranges[i] < 100.0]
        back_ranges = [msg.ranges[i] for i in back_indices if 0.01 < msg.ranges[i] < 100.0]
        self.front_clear = not any(r < self.safety_distance for r in front_ranges)
        self.back_clear = not any(r < self.safety_distance for r in back_ranges)
        if not self.front_clear and self.linear_speed > 0:
            self.stop_robot()
        elif not self.back_clear and self.linear_speed < 0:
            self.stop_robot()

    # Envia comandos de movimento para o robô
    def move_robot(self):
        if not self.killed:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
            print(f"Movendo: velocidade linear={self.linear_speed} m/s, velocidade angular={self.angular_speed} rad/s")

    # Para o robô
    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Parando o robô.")

    def increase_linear_speed(self):
        if self.front_clear:
            self.linear_speed += 0.1
            self.move_robot()
        else:
            self.stop_robot()

    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed -= 0.1
            self.move_robot()
        else:
            self.stop_robot()

    def increase_angular_speed(self):
        self.angular_speed += 0.1
        self.move_robot()

    def decrease_angular_speed(self):
        self.angular_speed -= 0.1
        self.move_robot()

    def kill_switch(self):
        self.killed = True
        self.stop_robot()

    def start_switch(self):
        self.killed = False

# Classe que ouve mensagens do tópico 'chatter'
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        # Cria um subscriber para ouvir mensagens do tópico 'chatter'
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # Evita aviso de variável não utilizada

    # Callback para processar mensagens recebidas
    def listener_callback(self, msg):
        timestamp, jpg_as_text = msg.data.split('|', 1)
        timestamp = float(timestamp)
        current_time = time.time()
        latency = current_time - timestamp
        # Decodifica a imagem recebida
        jpg_original = base64.b64decode(jpg_as_text)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
        if img is not None:
            print("Imagem recebida e decodificada.")
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)
            img_bytes = io.BytesIO()
            pil_image.save(img_bytes, format="JPEG")
            img_bytes.seek(0)
            # Coloca a imagem e a latência na fila para atualização da UI
            if not ui_queue.full():
                ui_queue.put((img_bytes, latency))
        else:
            self.get_logger().error('Não foi possível decodificar a imagem')

# Função para inicializar os nós ROS
@st.cache_resource
def init_ros_nodes():
    rclpy.init(domain_id=86)  # Inicializa o sistema ROS com o domain_id 86
    robot_controller = RobotController()
    listener = Listener()
    # Cria uma thread para executar os nós ROS
    executor_thread = threading.Thread(target=spin_nodes, args=(robot_controller, listener), daemon=True)
    executor_thread.start()
    return robot_controller, listener

# Função para executar os nós ROS em um loop
def spin_nodes(robot_controller, listener):
    while rclpy.ok():
        rclpy.spin_once(robot_controller, timeout_sec=0.01)
        rclpy.spin_once(listener, timeout_sec=0.01)

# Função para atualizar a interface do usuário
def ui_update():
    ctx = get_script_run_ctx()
    add_script_run_ctx(threading.current_thread(), ctx)
    while True:
        try:
            img_bytes, latency = ui_queue.get(timeout=0.1)
            st.session_state.img_bytes = img_bytes
            st.session_state.latency = latency
        except queue.Empty:
            pass

st.set_page_config(layout="wide")

if 'img_bytes' not in st.session_state:
    st.session_state.img_bytes = None
    st.session_state.latency = None

if 'ui_thread_started' not in st.session_state:
    ui_thread = threading.Thread(target=ui_update, daemon=True)
    add_script_run_ctx(ui_thread)
    ui_thread.start()
    st.session_state.ui_thread_started = True

# Inicializa os nós ROS
robot_controller, listener = init_ros_nodes()
frame_holder = st.empty()
latency_placeholder = st.empty()

col1, col2, col3 = st.columns([1, 2, 1])
with col1:
    if st.button("Desligar", key="kill_switch"):
        robot_controller.kill_switch()
with col2:
    st.write("")
    st.write("")
    st.write("")
    if st.button("W", key="increase_linear"):
        robot_controller.increase_linear_speed()
    if st.button("S", key="decrease_linear"):
        robot_controller.decrease_linear_speed()
with col3:
    st.write("")
    st.write("")
    st.write("")
    if st.button("A", key="increase_angular"):
        robot_controller.increase_angular_speed()
    if st.button("D", key="decrease_angular"):
        robot_controller.decrease_angular_speed()

# Loop de atualização para a exibição da imagem e da latência
while True:
    if st.session_state.img_bytes is not None:
        frame_holder.image(st.session_state.img_bytes, caption="Transmissão da Webcam", use_column_width=True, output_format='JPEG')
        latency_placeholder.text(f"Latência: {st.session_state.latency:.4f} segundos")
    time.sleep(0.01)
