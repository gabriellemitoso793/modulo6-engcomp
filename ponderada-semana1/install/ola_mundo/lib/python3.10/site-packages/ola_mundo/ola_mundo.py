import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
import math


class DrawHeart(Node):

    def __init__(self):
        super()._init_('draw_heart')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_client(Spawn, '/spawn')
        self.create_client(Kill, '/kill')
        self.create_client(SetPen, '/turtle1/set_pen')

    async def draw_heart(self):
        
        await self.set_pen(255, 0, 0, 3, 0)  # Cor vermelha, largura 3
        await self.move_to(5.0, 5.0)
        await self.draw_heart_shape()

    async def set_pen(self, r, g, b, width, off):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de set_pen não está disponível. Tentando novamente...')
        future = client.call_async(request)
        await future

    async def move_to(self, x, y):
        msg = Twist()
        msg.linear.x = x - 5.0
        msg.linear.y = y - 5.0  
        self.publisher_.publish(msg)
        await rclpy.spin_once(self) 

    async def draw_heart_shape(self):
        await self.draw_half_heart(1)
        await self.draw_half_heart(-1)

    async def draw_half_heart(self, direction):
        radius = 2.0
        start_angle = math.radians(90)
        end_angle = math.radians(270)
        angle = start_angle
        arc_length = 1.5 * radius * math.pi
        steps = 100
        step_size = (end_angle - start_angle) / steps

        for _ in range(steps):
            msg = Twist()
            msg.linear.x = radius * math.cos(angle) * direction
            msg.linear.y = radius * math.sin(angle)
            self.publisher_.publish(msg)
            await rclpy.spin_once(self)
            angle += step_size


def main(args=None):
    rclpy.init(args=args)
    draw_node = DrawHeart()

    try:
        rclpy.spin_until_future_complete(draw_node.draw_heart())
    except KeyboardInterrupt:
        pass

    draw_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()