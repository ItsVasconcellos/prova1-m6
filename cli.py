from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
import time
import typer


app = typer.Typer()

#Classe para criar uma tartaruga e manipular a mesma
class DrawingTurtle(Node):
    def __init__(self,turtle_name):
        super().__init__(turtle_name)
        self.turtle_name = turtle_name
        self.spawn_turtle = self.create_client(Spawn, '/spawn')
        self.kill_turtle = self.create_client(Kill, '/kill')
        self.set_pen = self.create_client(SetPen, f'/{self.turtle_name}/set_pen')
        self.draw = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel',10)
        self.command = self.create_subscription("/vel", list, self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.vx_deque = deque()
        self.vy_deque = deque()
        self.vt_deque = deque()
        self.t_deque = deque()
        
    # Função para remover a tartaruga original do node
    def kill_original(self):
        kill = self.kill_turtle.call_async(Kill.Request(name='turtle1'))
        rclpy.spin_until_future_complete(self, kill)

    # Função para criar uma nova tartaruga
    def spawn(self):
        spawn = self.spawn_turtle.call_async(Spawn.Request(x=5.5, y=5.5, theta=0.0, name=self.turtle_name))
        rclpy.spin_until_future_complete(self, spawn)
        new_turtle_name     = spawn.result().name
        print(new_turtle_name)

    # Função para remover a tartaruga
    def kill(self,name):
        kill = self.kill_turtle.call_async(Kill.Request(name=name))
        rclpy.spin_until_future_complete(self, kill)

    # Função para setar a cor da caneta
    def set_pen_color(self):
        set_pen = self.set_pen.call_async(SetPen.Request(r=20, g=255, b=20, width=1, off=0))
        rclpy.spin_until_future_complete(self, set_pen)


    def listener_callback(self, msg):
        publish_msg = Twist()
        publish_msg.linear.x = msg.vx 
        publish_msg.linear.y = msg.vy
        publish_msg.angular.z = msg.vt
        self.draw(publish_msg)

    def add_deque(self,vx,vy,vt,t):
        self.vx_deque.append(vx)
        self.vy_deque.append(vy)
        self.vt_deque.append(vt)
        self.t_deque.append(t)

    # Função para desenhar um círculo
    def draw_circle(self):
        msg = Twist()
        msg.linear.x = self.vx_deque.popleft()
        msg.linear.y = self.vy_deque.popleft()
        msg.angular.z = self.vt_deque.popleft()
        for _ in range(self.t_deque.popleft()):
            self.draw.publish(msg)
            time.sleep(1)
        
@app.comand()
def mover(vx: int, vy:int, vt:int = 0, t:int = 1000):
    rclpy.init()
    node = DrawingTurtle("PalmeirasTurtle")
    node.add_deque(vx,vy,vt,t)
    for _ in node.vx_deque :
        node.draw()

# Executa a aplicação
if __name__ == "__main__":
    app()
