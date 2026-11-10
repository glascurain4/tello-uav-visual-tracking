#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_control_node')
        
        # Suscribirse a la información visual
        self.subscription = self.create_subscription(
            Point,
            '/tello/target_info',
            self.control_callback,
            10
        )
        
        # Publicar comandos de movimiento al Tello
        self.cmd_vel_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)

        # --- PARÁMETROS DE CONTROL Y RECOMENDACIONES TÉCNICAS ---
        self.threshold_x = 25.0       # Zona muerta eje X (px)
        self.threshold_y = 25.0       # Zona muerta eje Y (px)
        self.target_area = 5000.0    # Área deseada (distancia objetivo)
        self.area_threshold = 2800.0  # Zona muerta del área (distancia)

        # Límites máximos de velocidad (Seguridad)
        self.MAX_LINEAR_SPEED = 0.2   # m/s aproximados según driver
        self.MAX_ANGULAR_SPEED = 0.4  # rad/s

        # Ganancias Proporcionales para suavizado
        self.K_yaw = -0.002	#Cambio dse signo para invertir direción en giro
        self.K_up_down = -0.002       # Negativo si la cámara invierte dirección
        self.K_forward = 0.00002

        self.get_logger().info("Nodo de Control Tello Inicializado.")

    def control_callback(self, msg):
        twist = Twist()

        # Detener en dado caso no se detecte 
        if msg.z == 0.0:
            self.get_logger().warn("¡Objetivo perdido! Deteniendo motores.")
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # Procesamineto zonas muertas y zonas seguras
        error_x = msg.x
        error_y = msg.y
        current_area = msg.z

        # Control del Giro sobre eje Z
        if abs(error_x) > self.threshold_x:
            twist.angular.z = - (error_x * self.K_yaw)
        else:
            twist.angular.z = 0.0

        # Control de Altura Up / Down, linear.z
        if abs(error_y) > self.threshold_y:
            twist.linear.z = error_y * self.K_up_down
        else:
            twist.linear.z = 0.0

        # Control de Distancia, Forward / Backward, linear.x
        area_error = self.target_area - current_area
        if abs(area_error) > self.area_threshold:
            twist.linear.x = area_error * self.K_forward
        else:
            twist.linear.x = 0.0

        # Acotar a velocidades máximas seguras
        twist.linear.x = max(min(twist.linear.x, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        twist.linear.z = max(min(twist.linear.z, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        twist.angular.z = max(min(twist.angular.z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        
        # Asegurar movimiento lateral estático
        twist.linear.y = 0.0 

        # Publicar comando de velocidad
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TelloControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Enviar comando de parada antes de morir
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
