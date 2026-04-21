#Imports
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


# Class Definition
class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('Trajectory_node')

        #Parámetros
        self.declare_parameter('coordenadas_x', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) 
        self.declare_parameter('coordenadas_y', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY)         
        self.declare_parameter('velocidad',     rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) 
        self.declare_parameter('area', 0.0)

        self.x = list(self.get_parameter('coordenadas_x').get_parameter_value().double_array_value)
        self.y = list(self.get_parameter('coordenadas_y').get_parameter_value().double_array_value)
        self.v = list(self.get_parameter('velocidad').get_parameter_value().double_array_value)
        self.a = self.get_parameter('area').value

        # Estado interno
        self.waypoints = []  # Puntos validos
        self.current_idx = 0 # Indice del setpoint actual
        self.waiting = False # True mientras espera el next_point del controller

        # Pub/Sub
        self.pub_setpoint = self.create_publisher(Point, 'setpoint',10)
        self.create_subscription(Point, 'next_point', self.next_point_cb, 10)

        # Valida y contruye waypoints 
        if self.calculate_path():
            self.get_logger().info(f'Trayectoria lista: {len(self.waypoints)}')
            self._publish_next_setpoint()
        else:
            self.get_logger().error('Trayectoria inválida. Revisa los parámetros. ')



    def calculate_path(self):
        """
        Verifica que los puntos dados esten dentro del area de trabajo
        Y contruye selg.waypoints
        """

        self.get_logger().info(f" Iniciando Comprobacion")

        # Definicion del area de trabajo
        limite = self.a/2

        # Velocidad Lineal
        v_min = 0.025
        v_max = 0.38

        # Se verifica si el vector esta vacio
        if len(self.x)==0:
            self.get_logger().error(" No se a dado un vector de coordenadas")
            return False

        if not self.x:
            self.get_logger().error(" Vector de coordenadas vacio")
            return False
        
        if len(self.x) != len(self.y):
            self.get_logger().error(" Vector 'x' 'y' de distinto tamaño")
            return False
        
        if len(self.x) != len(self.v):
            self.get_logger().error(" Vector de velocidad de distinto tamaño que las coordenadas")
            return False
        
        prev = np.array([0.0, 0.0])   # origen

        for i, (xi, yi, vi) in enumerate(zip(self.x, self.y, self.v)):

            if not (-limite <= xi <= limite and -limite <= yi <= limite):
                self.get_logger().error(f'Punto {i} ({xi}, {yi}) fuera del área {self.a}m.')
                return False 
            if not (v_min <= vi <= v_max):
                self.get_logger().error(f'Velocidad {i} = {vi} fuera del rango permitido.')
                return False
            
            curr = np.array([xi, yi])
            dist = float(np.linalg.norm(curr-prev))
            t_est = dist / vi
            self.get_logger().info(f'Punto {i}: ({xi:.3f}, {yi:.3f})  v={vi:.3f} m/s  t≈{t_est:.2f}s')

            p = Point()
            p.x = xi
            p.y = yi
            p.z = 0.0
            self.waypoints.append(p)
            prev = curr
        return True

    def _publish_next_setpoint(self):

        """Publica el setpoint actual y marca que estamos esperando."""

        if self.current_idx >= len(self.waypoints):
            self.get_logger().info("Todos los setpoints enviados. Trayectoria completa.")
            return

        # Se instancia el custom msg
        msg = self.waypoints[self.current_idx]
        self.pub_setpoint.publish(msg)
        self.get_logger().info(f'Setpoint {self.current_idx} publicado: ({msg.x:.3f}, {msg.y:.3f})')
        
        self.waiting = True
    
    def next_point_cb(self, msg: Point):

        "Callback: el controller llegó al punto, manda el siguiente."

        if not self.waiting:
            return
        
        self.get_logger().info(f'next_point recibido: ({msg.x:.3f}, {msg.y:.3f})')
        self.waiting =  False
        self.current_idx += 1
        self._publish_next_setpoint()


def main(args=None):
    rclpy.init(args=args)
    nodo = TrajectoryNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
       
#Execute Node
if __name__ == '__main__':
    main()