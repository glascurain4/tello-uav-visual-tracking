#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
# --- CAMBIO 1: Importar los perfiles de QoS necesarios ---
from rclpy.qos import QoSProfile, ReliabilityPolicy 
from cv_bridge import CvBridge
import cv2
import numpy as np

class TelloVisionNode(Node):
    def __init__(self):
        super().__init__('tello_vision_node')
        
        # definir el perfil para el video 
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Suscripción al video del dron
        self.subscription = self.create_subscription(
            Image,
            '/tello/image_raw',  
            self.image_callback,
            qos_profile  # --- CAMBIO 3: Cambiar el "10" por el qos_profile ---
        )
        
        # Publicador de información del objetivo
        self.publisher_ = self.create_publisher(Point, '/tello/target_info', 10)
        self.bridge = CvBridge()

        # Configuración de Ventanas y Trackbars OpenCV
        self.WINDOW_NAME = "Tello Camera"
        self.TRACKBARS_NAME = "Trackbars"
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW_NAME, 800, 600)
        cv2.namedWindow(self.TRACKBARS_NAME)

        # Valores iniciales HSV (cara azul del cubo, se calibran en vivo con las trackbars)
        cv2.createTrackbar("H_min", self.TRACKBARS_NAME, 82, 179, lambda x: None)
        cv2.createTrackbar("H_max", self.TRACKBARS_NAME, 179, 179, lambda x: None)
        cv2.createTrackbar("S_min", self.TRACKBARS_NAME, 195, 255, lambda x: None)
        cv2.createTrackbar("S_max", self.TRACKBARS_NAME, 255, 255, lambda x: None)
        cv2.createTrackbar("V_min", self.TRACKBARS_NAME, 0, 255, lambda x: None)
        cv2.createTrackbar("V_max", self.TRACKBARS_NAME, 137, 255, lambda x: None)

        # Dimensiones de procesamiento
        self.width = 400
        self.height = 300
        self.area_min = 500

        self.get_logger().info("Nodo de Visión Tello Inicializado.")

    def image_callback(self, msg):
        # Convertir mensaje de ROS a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (self.width, self.height))

        # Procesamiento HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        h_min = cv2.getTrackbarPos("H_min", self.TRACKBARS_NAME)
        h_max = cv2.getTrackbarPos("H_max", self.TRACKBARS_NAME)
        s_min = cv2.getTrackbarPos("S_min", self.TRACKBARS_NAME)
        s_max = cv2.getTrackbarPos("S_max", self.TRACKBARS_NAME)
        v_min = cv2.getTrackbarPos("V_min", self.TRACKBARS_NAME)
        v_max = cv2.getTrackbarPos("V_max", self.TRACKBARS_NAME)

        hsv_min = np.array([h_min, s_min, v_min])
        hsv_max = np.array([h_max, s_max, v_max])

        mask = cv2.inRange(hsv_frame, hsv_min, hsv_max)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cx = self.width // 2
        cy = self.height // 2

        # líneas de centro de referencia
        cv2.line(frame, (cx, 0), (cx, self.height), (255, 0, 0), 1)
        cv2.line(frame, (0, cy), (self.width, cy), (255, 0, 0), 1)

        largest_contour = None
        largest_area = 0
        target_center_x = 0
        target_center_y = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.area_min:
                if area > largest_area:
                    largest_area = area
                    largest_contour = contour

        info_msg = Point()

        if largest_contour is not None:
            x, y, w, h = cv2.boundingRect(largest_contour)
            target_center_x = x + w // 2
            target_center_y = y + h // 2

            # Calcular errores con respecto al centro de la imagen
            error_x = float(target_center_x - cx)
            error_y = float(target_center_y - cy)

            # Preparar mensaje para el controlador
            info_msg.x = error_x
            info_msg.y = error_y
            info_msg.z = float(largest_area)

            # UI de diagnóstico
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.circle(frame, (target_center_x, target_center_y), 7, (0, 0, 255), -1)
            cv2.putText(frame, f"Area: {int(largest_area)}", (x, y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # Objetivo perdido: enviar todo a 0, deternlo
            info_msg.x = 0.0
            info_msg.y = 0.0
            info_msg.z = 0.0
            cv2.putText(frame, "TARGET LOST", (self.width // 2 - 100, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        # Publicar los datos calculados
        self.publisher_.publish(info_msg)

        # Desplegar interfaces de OpenCV
        cv2.imshow(self.WINDOW_NAME, frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TelloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
