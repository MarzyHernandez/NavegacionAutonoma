#!/usr/bin/env python3
# Autor: Mariana Marzayani Hernandez Jurado

#Librerias
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import random
import math
import subprocess
import time
import threading
import sys
import os  # El módulo 'os' es util para controlar el proceso de TurtleSim

# Clase Nodo 
class MyNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle Controller Node inicioado")

        self.turtle_pose = None
        self.target_x = random.uniform(0.1, 10.0)
        self.target_y = random.uniform(0.1, 10.0)
        self.approx_x = 0.0  # Coordenada X aproximada del objetivo
        self.approx_y = 0.0  # Coordenada Y aproximada del objetivo
        self.search_radius = 1.5  # Radio de búsqueda del objetivo

        self.pub_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_turtle_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

        self.is_running = True
        self.lock = threading.Lock()
        self.turtle_counter = 1  # Contador para nombres únicos de objetivos 

        self.timer = self.create_timer(0.1, self.move_turtle)
        self.init_keyboard_input_thread()

        self.spawn_turtle()
        
        # Variable de estado para seguir el progreso hacia el objetivo
        self.is_approaching = False
        # Variable para rastrear si se ha mostrado el mensaje de éxito
        self.success_message_shown = False

    def pose_callback(self, msg):
        self.turtle_pose = msg

    def move_turtle(self):
        if self.turtle_pose is not None:
            if self.approx_x == 0.0 and self.approx_y == 0.0:
                # Generar una coordenada aproximada aleatoria del objetivo
                # Coordenada que nos ayudara a buscar el ARuco
                self.approx_x = self.target_x + random.uniform(-1.0, 1.0)
                self.approx_y = self.target_y + random.uniform(-1.0, 1.0)
            
            distance = math.sqrt((self.approx_x - self.turtle_pose.x) ** 2 + (self.approx_y - self.turtle_pose.y) ** 2)
            angle = math.atan2(self.approx_y - self.turtle_pose.y, self.approx_x - self.turtle_pose.x)
            cmd_vel_msg = Twist()

            if distance <= self.search_radius:
                # Dentro del radio de búsqueda, ahora vamos hacia el objetivo real ARuco
                self.is_approaching = True
                distance_to_target = math.sqrt((self.target_x - self.turtle_pose.x) ** 2 + (self.target_y - self.turtle_pose.y) ** 2)
                angle_to_target = math.atan2(self.target_y - self.turtle_pose.y, self.target_x - self.turtle_pose.x)
                cmd_vel_msg.linear.x = min(1.0, distance_to_target)
                cmd_vel_msg.angular.z = 4.0 * (angle_to_target - self.turtle_pose.theta)
            else:
                cmd_vel_msg.linear.x = min(1.0, distance)
                cmd_vel_msg.angular.z = 4.0 * (angle - self.turtle_pose.theta)

            self.pub_cmd_vel.publish(cmd_vel_msg)

            # Imprimir coordenadas actuales y objetivo en todo momento
            if not self.success_message_shown:
                self.print_coordinates()

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.approx_x
            marker.pose.position.y = self.approx_y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)

            # Verificar si la tortuga ha llegado al objetivo
            # Margen de 0.1
            if self.is_approaching and distance_to_target < 0.1:
                self.success_message_shown = True
                self.get_logger().info("¡El robot llegó al objetivo BIEN!")
                self.is_approaching = False
                self.approx_x = 0.0
                self.approx_y = 0.0

    def print_coordinates(self):
        # Imprimir coordenadas actuales y objetivo en todo momento
        if self.turtle_pose is not None:
            self.get_logger().info("Coordenadas actuales: x=%.2f, y=%.2f" % (self.turtle_pose.x, self.turtle_pose.y))
        if self.is_approaching:
            self.get_logger().info("Objetivo: x=%.2f, y=%.2f (Aproximando)" % (self.target_x, self.target_y))
        else:
            self.get_logger().info("Objetivo: x=%.2f, y=%.2f" % (self.target_x, self.target_y))

    def init_keyboard_input_thread(self):
        keyboard_thread = threading.Thread(target=self.keyboard_input_thread)
        keyboard_thread.daemon = True
        keyboard_thread.start()

    def keyboard_input_thread(self):
        while self.is_running:
            key_input = input("Presiona 1 para mover la tortuga o 2 para salir: ")
            if key_input == "1":
                self.lock.acquire()
                self.target_x = random.uniform(0.1, 10.0)
                self.target_y = random.uniform(0.1, 10.0)
                self.approx_x = 0.0
                self.approx_y = 0.0
                self.success_message_shown = False  # Reiniciar el mensaje de éxito
                self.spawn_turtle()
                self.lock.release()
            elif key_input == "2":
                self.is_running = False
                # Cerrar la ventana de TurtleSim
                os.system("pkill -f turtlesim_node")
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)

    def spawn_turtle(self):
        try:
            spawn_client = self.create_client(Spawn, 'spawn')
            while not spawn_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Esperando al servicio de generacion...')
            request = Spawn.Request()
            request.x = self.target_x
            request.y = self.target_y
            request.theta = 0.0
            request.name = "red_turtle_{}".format(self.turtle_counter)
            self.turtle_counter += 1
            spawn_client.call_async(request)
            self.get_logger().info('Nueva tortuga en ({:.2f}, {:.2f})'.format(request.x, request.y))
        except Exception as e:
            self.get_logger().error('Error mientras se generaba una nueva tortuga: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    subprocess.Popen(["ros2", "run", "turtlesim", "turtlesim_node"])
    time.sleep(2)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

# Clase principal

if __name__ == '__main__':
    main()

