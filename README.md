## Una forma facil y efectiva de correr el programa
Al tratarse de un proyecto desarrollado en el ambiente de Ros2, primero debemos asegurarnos de tener todas las librerias necesarias para evitar errores. 
Este poryecto usa las siguientes. 
+  rclpy
+  random
+   math
+   subbprocess
+   time
+   threading
+   sys
+   os

Una vez que estemos seguros de tener todos los paquetes, podemos proceder a correr el programa.
1. Lo primero que debemos de hacer es abrir una terminal en Ubuntu 22.04
2. Posteriorimente debemos de entrar al folder que contiene la carpeta que contiene el archivo node.py
   Por ejemplo: ~/ros2_ws/src/my_robot_controller/my_robot_controller
3. Luego debemos de correr el archivo node.py
   De la siguiente forma: ./node.py

Cosas importantes:
+  Para que ir a un nuevo objetivo aleatorio, debemos de posicionarnos en la terminal, presionar uno (1) y dar enter
+  Para detener por completo la simulacion, debemos de posicionarnos en la terminal, presionar dos (2) y dar enter

## Funcionamiento bloque a bloque
Bloque 1: Importación de módulos

    #!/usr/bin/env python3
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
    import os

Este bloque importa los módulos necesarios para el funcionamiento del programa. Algunos de los módulos son específicos de ROS 2 y se utilizan para interactuar con los nodos y mensajes de ROS 2, mientras que otros se utilizan para operaciones generales como matemáticas y manipulación del sistema.

Bloque 2: Clase MyNode

    class MyNode(Node):
        def __init__(self):
            super().__init__("turtle_controller")
            self.get_logger().info("Turtle Controller Node started")

Se define una clase llamada MyNode que hereda de Node, una clase base proporcionada por ROS 2 para crear nodos ROS. En el constructor __init__, se inicia el nodo con el nombre "turtle_controller" y se muestra un mensaje de inicio.

Bloque 3: Variables y configuración inicial


        self.turtle_pose = None
        self.target_x = random.uniform(0.1, 10.0)
        self.target_y = random.uniform(0.1, 10.0)
        self.approx_x = 0.0
        self.approx_y = 0.0
        self.search_radius = 1.5

Se inicializan varias variables de clase, incluyendo la posición de la tortuga, las coordenadas del objetivo (inicializadas aleatoriamente), coordenadas aproximadas del objetivo, y un radio de búsqueda del objetivo.

Bloque 4: Publicadores y suscriptores

        self.pub_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_turtle_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

Se crean un publicador y un suscriptor ROS. El publicador se utiliza para enviar comandos de velocidad (Twist) a la tortuga, mientras que el suscriptor se utiliza para recibir la posición de la tortuga (Pose) desde el simulador TurtleSim.

Bloque 5: Publicador para marcadores


        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

Se crea un publicador para enviar marcadores visuales al simulador. Esto se utiliza para visualizar la posición aproximada del objetivo.

Bloque 6: Variables de control


        self.is_running = True
        self.lock = threading.Lock()
        self.turtle_counter = 1

Se inicializan variables de control, como is_running para indicar si el programa está en ejecución, lock para gestionar el acceso concurrente a ciertas secciones críticas del código y turtle_counter para asignar nombres únicos a las tortugas.

Bloque 7: Temporizador y configuración adicional

        self.timer = self.create_timer(0.1, self.move_turtle)
        self.init_keyboard_input_thread()
        self.spawn_turtle()
        self.is_approaching = False
        self.success_message_shown = False

 Se crea un temporizador que llama a la función move_turtle cada 0.1 segundos para controlar el movimiento de la tortuga.
    Se inicializa un hilo para capturar la entrada del teclado en segundo plano.
    Se crea una nueva tortuga en el simulador.

Bloque 8: Callback de posición


    def pose_callback(self, msg):
        self.turtle_pose = msg

Se define una función de devolución de llamada (pose_callback) que se utiliza para actualizar la posición de la tortuga en función de los mensajes de posición recibidos.

Bloque 9: Función move_turtle

    def move_turtle(self):
        # ...

Esta función controla el movimiento de la tortuga hacia el objetivo. Calcula la velocidad lineal y angular necesaria en función de la posición actual y el objetivo.

Bloque 10: Función print_coordinates


    def print_coordinates(self):
        # ...

Esta función imprime las coordenadas actuales de la tortuga y las coordenadas del objetivo en el registro de ROS.

Bloque 11: Inicialización del hilo de entrada del teclado


    def init_keyboard_input_thread(self):
        keyboard_thread = threading.Thread(target=self.keyboard_input_thread)
        keyboard_thread.daemon = True
        keyboard_thread.start()

Se inicializa el hilo que captura la entrada del teclado y llama a la función keyboard_input_thread.

Bloque 12: Hilo de entrada del teclado


    def keyboard_input_thread(self):
        while self.is_running:
            key_input = input("Presiona 1 para mover la tortuga o 2 para salir: ")
            if key_input == "1":
                # ...
            elif key_input == "2":
                # ...

 Este hilo espera la entrada del teclado y permite al usuario mover la tortuga o salir del programa.

Bloque 13: Función spawn_turtle


    def spawn_turtle(self):
        try:
            # ...
        except Exception as e:
            self.get_logger().error('Error while spawning the turtle: {}'.format(str(e)))

Esta función se utiliza para crear una nueva tortuga en el simulador TurtleSim en una posición aleatoria.

Bloque 14: Función main


    def main(args=None):
        rclpy.init(args=args)
        subprocess.Popen(["ros2", "run", "turtlesim", "turtlesim_node"])
        time.sleep(2)
        node = MyNode()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

La función main se utiliza para iniciar el nodo ROS, abrir el simulador TurtleSim, crear una instancia de MyNode, iniciar el bucle de eventos ROS y, finalmente, cerrar todo cuando se termine la ejecución del programa.

## Calculos

La parte matemática principal se encuentra en la función move_turtle, que calcula la velocidad lineal y angular necesaria para que la tortuga se mueva hacia un objetivo.

El objetivo de la tortuga se divide en dos partes:

+ Coordenadas aproximadas del objetivo (approx_x y approx_y): Estas coordenadas se establecen inicialmente de manera aleatoria dentro de un radio de búsqueda alrededor del objetivo real.

+ Objetivo real (target_x y target_y): Estas son las coordenadas del objetivo real hacia el cual la tortuga se está moviendo.

Para calcular la velocidad de la tortuga, se utilizan las siguientes fórmulas matemáticas:

Distancia (distance) entre la posición actual de la tortuga y las coordenadas aproximadas del objetivo:

    distance = math.sqrt((approx_x - turtle_pose.x) ** 2 + (approx_y - turtle_pose.y) ** 2)

Ángulo (angle) entre la posición actual de la tortuga y las coordenadas aproximadas del objetivo:

    angle = math.atan2(approx_y - turtle_pose.y, approx_x - turtle_pose.x)

Distancia al objetivo real (distance_to_target) y ángulo al objetivo real (angle_to_target):

    distance_to_target = math.sqrt((target_x - turtle_pose.x) ** 2 + (target_y - turtle_pose.y) ** 2)
    angle_to_target = math.atan2(target_y - turtle_pose.y, target_x - turtle_pose.x)

Luego, se crea un mensaje de velocidad (cmd_vel_msg) que se publica para controlar la tortuga. La velocidad lineal (cmd_vel_msg.linear.x) y la velocidad angular (cmd_vel_msg.angular.z) se calculan en función de las distancias y ángulos mencionados anteriormente. El código también considera el estado de la tortuga (si está dentro del radio de búsqueda o si se está aproximando al objetivo real) para ajustar las velocidades.

Diagrama conceptual:


       |   
       |   
       |         Target (target_x, target_y)
       |          /
       |         /
       |        /
       |       /
       |      /     
       |     / 
       |    /
       |   /         
       |  /
       | /
       |/
       |______________________
       Turtle Position (turtle_pose.x, turtle_pose.y)

En este diagrama, la tortuga se encuentra en su posición actual (turtle_pose.x, turtle_pose.y) y se mueve hacia un objetivo (target_x, target_y). Las coordenadas aproximadas del objetivo se definen dentro de un radio de búsqueda alrededor del objetivo real.
De esta forma, la tortuga primero se acerca a una coordenada aproximada y busca en un radio definido su objetivo.
