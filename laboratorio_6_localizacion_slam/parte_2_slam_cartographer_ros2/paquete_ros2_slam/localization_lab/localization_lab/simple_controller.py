import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class ReactiveController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        # 1. PUBLICADOR: Para enviar las velocidades a las ruedas
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # 2. SUSCRIPTOR: Para escuchar los datos del sensor LiDAR
        # Usamos qos_profile_sensor_data porque el láser envía muchos datos muy rápido
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        
        # 3. TEMPORIZADOR: Ejecuta el movimiento cada 0.1 segundos (10 Hz)
        self.timer = self.create_timer(0.1, self.move_robot)
        
        # Variables de estado interno
        self.distancia_frontal = 1.0  # Iniciamos asumiendo que el frente está libre (1 metro)
        self.step_count = 0
        
        self.get_logger().info("Controlador Reactivo Iniciado: Evasión de obstáculos activada.")

    def scan_callback(self, msg):
        """Esta función se ejecuta automáticamente cada vez que el LiDAR envía un dato."""
        
        # El LiDAR de Turtlebot3 entrega 360 valores (de 0 a 359 grados).
        # El grado 0 es exactamente al frente. 
        # Tomamos un "cono" visual de 60 grados: 30 grados a la izquierda y 30 a la derecha.
        rangos_frente = msg.ranges[-30:] + msg.ranges[:30]
        
        # Filtramos errores del sensor (distancias infinitas o ceros falsos)
        rangos_validos = [r for r in rangos_frente if 0.1 < r < 10.0]
        
        if rangos_validos:
            # Encontramos cuál es el punto más cercano al robot dentro de ese cono frontal
            self.distancia_frontal = min(rangos_validos)
        else:
            self.distancia_frontal = 10.0  # Si no hay nada, asumimos vía libre

    def move_robot(self):
        """Esta función toma decisiones de movimiento basadas en la lectura del LiDAR."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # DEFINIMOS EL UMBRAL DE SEGURIDAD (50 cm)
        distancia_seguridad = 0.50  
        
        if self.distancia_frontal < distancia_seguridad:
            # 🚨 OBSTÁCULO DETECTADO: El robot se detiene y gira sobre su eje
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.6  # Girar hacia la izquierda a 0.6 rad/s
            estado = f"🚨 OBSTACULO a {self.distancia_frontal:.2f}m -> Girando evasivamente..."
        else:
            # ✅ CAMINO LIBRE: El robot avanza en línea recta
            cmd.twist.linear.x = 0.2
            cmd.twist.angular.z = 0.0
            estado = f"✅ Camino libre ({self.distancia_frontal:.2f}m) -> Avanzando"

        # Publicamos el comando de movimiento
        self.publisher_.publish(cmd)
        
        # Imprimimos en consola 1 vez por segundo para no saturar la pantalla
        self.step_count += 1
        if self.step_count % 10 == 0:
            self.get_logger().info(estado)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
