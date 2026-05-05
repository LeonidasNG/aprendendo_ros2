#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.get_logger().info('Nó robot_control inicializado!')

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.distancia_objetivo = 1.0

        self.pgain = 0.5
        self.igain = 0.01
        self.dgain = 0.1

        self.error = 0.0
        self.integral = 0.0
        self.derivada = 0.0
        self.erro_anterior = 0.0
        self.dt = 0.1
        self.last_time = self.get_clock().now()

        self.dist_direita = float('inf')
        self.dist_frente = float('inf')
        self.laser_ranges = []
        self.lx = 0.0
        self.az = 0.0


    def laser_callback(self, msg):
        self.laser_ranges = list(msg.ranges)

        # Direita = 270° → índice ~154 (confirmado)
        faixa = [d for d in self.laser_ranges[144:165] if 0 < d < 10.0]
        self.dist_direita = np.mean(faixa) if faixa else float('inf')

        # Frente = 0° → índices 0–10 e 350–359
        faixa_frente = [d for d in self.laser_ranges[0:11] + self.laser_ranges[350:360] if 0 < d < 10.0]
        self.dist_frente = np.mean(faixa_frente) if faixa_frente else float('inf')

        self.get_logger().info(
            f'Direita: {self.dist_direita:.2f}m | '
            f'Frente: {self.dist_frente:.2f}m | '
            f'Erro: {self.error:.2f}'
        )

        self.calcular_pid()
        self.publicar_velocidade()


    def calcular_pid(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        if 0 < dt < 1.0:
            self.dt = dt
        self.last_time = now

        if np.isinf(self.dist_direita) or np.isnan(self.dist_direita):
            self.lx = 0.3
            self.az = 0.0
            return

        self.error = self.distancia_objetivo - self.dist_direita

        self.integral += self.error * self.dt
        self.integral = np.clip(self.integral, -2.0, 2.0)

        self.derivada = (self.error - self.erro_anterior) / self.dt if self.dt > 0 else 0.0

        power = (self.pgain * self.error +
                 self.igain * self.integral +
                 self.dgain * self.derivada)

        if np.isnan(power):
            power = 0.0

        self.az = float(np.clip(power, -1.5, 1.5))
        self.lx = float(max(0.15, 0.45 - 0.1 * abs(self.error)))
        self.erro_anterior = self.error


    def publicar_velocidade(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if np.isinf(self.dist_frente) or np.isnan(self.dist_frente):
            msg.twist.linear.x = 0.3
            msg.twist.angular.z = 0.0

        elif self.dist_frente < 1.0:
            msg.twist.linear.x = 0.05
            msg.twist.angular.z = 1.5
            
            self.get_logger().info('Parede na frente! Girando esquerda...')

        else:
            msg.twist.linear.x = self.lx
            msg.twist.angular.z = self.az

        self.pub_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    robot = RobotControl()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info('Nó interrompido pelo usuário')
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
