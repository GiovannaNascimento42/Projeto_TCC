import math
import csv
import os
from datetime import datetime

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PIDPositionController(Node):

    def __init__(self):
        super().__init__('pid_position_controller')

        # ============================================================
        # PARÂMETROS
        # ============================================================

        self.declare_parameter('control_frequency', 20.0)

        # PID LINEAR
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.0)

        # PID ANGULAR
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.0)

        # Limites de velocidade
        self.declare_parameter('max_linear_velocity', 0.10)
        self.declare_parameter('max_angular_velocity', 0.40)

        # ============================================================
        # TOLERÂNCIAS DE PARADA
        # ============================================================

        # Erro máximo de posição para considerar que chegou
        self.declare_parameter('position_tolerance', 0.05)

        # Erro angular máximo para considerar que terminou
        self.declare_parameter('angle_tolerance', 0.05)

        # ============================================================
        # LIMITES DOS INTEGRAIS
        # ============================================================

        self.declare_parameter('integral_limit_linear', 0.5)
        self.declare_parameter('integral_limit_angular', 0.5)

        # ============================================================
        # OBJETIVO DO TESTE
        # ============================================================

        # Distância que o robô deve andar
        self.declare_parameter('goal_distance', 1.0)

        # Ângulo que o robô deve girar depois de andar
        # positivo = esquerda
        # negativo = direita
        self.declare_parameter('turn_angle_deg', 90.0)

        # ============================================================
        # ARMAR / DESARMAR
        # ============================================================

        self.declare_parameter('armed', False)

        # ============================================================
        # VARIÁVEIS DE ODOMETRIA
        # ============================================================

        self.x = None
        self.y = None
        self.theta = None

        self.have_odom = False

        # ============================================================
        # POSE INICIAL
        # ============================================================

        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None

        # ============================================================
        # OBJETIVO
        # ============================================================

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

        # ============================================================
        # ESTADO DO EXPERIMENTO
        # ============================================================

        # Fases:
        #
        # MOVE  -> anda até a posição
        # ROTATE -> gira até o ângulo
        # DONE  -> para
        #
        self.state = 'IDLE'

        self.was_armed = False
        self.goal_reached = False

        # ============================================================
        # PID
        # ============================================================

        self.integral_linear = 0.0
        self.integral_angular = 0.0

        self.previous_linear_error = 0.0
        self.previous_angular_error = 0.0

        self.previous_time = self.get_clock().now()

        # ============================================================
        # LOGGER
        # ============================================================

        self.log_file = None
        self.csv_writer = None
        self.experiment_start_time = None

        log_directory = os.path.expanduser(
            '~/pid_experiments'
        )

        os.makedirs(
            log_directory,
            exist_ok=True
        )

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        log_path = os.path.join(
            log_directory,
            f'pid_distance_turn_{timestamp}.csv'
        )

        self.log_file = open(
            log_path,
            'w',
            newline=''
        )

        self.csv_writer = csv.writer(
            self.log_file
        )

        self.csv_writer.writerow([
            'time',
            'state',
            'x',
            'y',
            'theta',
            'goal_x',
            'goal_y',
            'goal_theta',
            'position_error',
            'angle_error',
            'linear_cmd',
            'angular_cmd'
        ])

        self.log_file.flush()

        self.get_logger().info(
            f'Arquivo de experimento: {log_path}'
        )

        # ============================================================
        # ROS
        # ============================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        frequency = self.get_parameter(
            'control_frequency'
        ).value

        self.timer = self.create_timer(
            1.0 / frequency,
            self.control_loop
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'PID Distance + Turn Controller iniciado.'
        )

        self.get_logger().info(
            'Estado inicial: DESARMADO.'
        )

        self.get_logger().info(
            'Use: ros2 param set '
            '/pid_position_controller armed true'
        )

        self.get_logger().info(
            '========================================'
        )

    # ================================================================
    # ODOMETRIA
    # ================================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion -> yaw
        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.theta = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.have_odom = True

    # ================================================================
    # NORMALIZAÇÃO ANGULAR
    # ================================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ================================================================
    # INICIALIZA OBJETIVO
    # ================================================================

    def initialize_goal(self):

        if not self.have_odom:
            return False

        # ------------------------------------------------------------
        # GUARDA POSE INICIAL
        # ------------------------------------------------------------

        self.initial_x = self.x
        self.initial_y = self.y
        self.initial_theta = self.theta

        # ------------------------------------------------------------
        # PARÂMETROS
        # ------------------------------------------------------------

        distance = self.get_parameter(
            'goal_distance'
        ).value

        turn_angle_deg = self.get_parameter(
            'turn_angle_deg'
        ).value

        turn_angle_rad = math.radians(
            turn_angle_deg
        )

        # ------------------------------------------------------------
        # META DE POSIÇÃO
        #
        # Anda "distance" metros na direção inicial.
        # ------------------------------------------------------------

        self.goal_x = (
            self.initial_x
            + distance * math.cos(self.initial_theta)
        )

        self.goal_y = (
            self.initial_y
            + distance * math.sin(self.initial_theta)
        )

        # ------------------------------------------------------------
        # META DE ORIENTAÇÃO
        #
        # Exemplo:
        #
        # theta inicial = 0°
        # turn_angle = +90°
        #
        # theta final = 90°
        # ------------------------------------------------------------

        self.goal_theta = self.normalize_angle(
            self.initial_theta + turn_angle_rad
        )

        # ------------------------------------------------------------
        # RESET DOS PIDs
        # ------------------------------------------------------------

        self.integral_linear = 0.0
        self.integral_angular = 0.0

        self.previous_linear_error = 0.0
        self.previous_angular_error = 0.0

        # ------------------------------------------------------------
        # ESTADO
        # ------------------------------------------------------------

        self.state = 'MOVE'
        self.goal_reached = False

        self.experiment_start_time = (
            self.get_clock().now()
        )

        # ------------------------------------------------------------
        # LOG
        # ------------------------------------------------------------

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'NOVO EXPERIMENTO'
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'Pose inicial: '
            f'x={self.initial_x:.3f} '
            f'y={self.initial_y:.3f} '
            f'theta={math.degrees(self.initial_theta):.2f}°'
        )

        self.get_logger().info(
            f'Distância: {distance:.3f} m'
        )

        self.get_logger().info(
            f'Giro: {turn_angle_deg:.2f}°'
        )

        self.get_logger().info(
            f'Meta posição: '
            f'x={self.goal_x:.3f} '
            f'y={self.goal_y:.3f}'
        )

        self.get_logger().info(
            f'Meta orientação: '
            f'{math.degrees(self.goal_theta):.2f}°'
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'FASE 1: ANDAR'
        )

        self.get_logger().info(
            '========================================'
        )

        return True

    # ================================================================
    # PUBLICA ZERO
    # ================================================================

    def publish_stop(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ================================================================
    # CONTROLADOR
    # ================================================================

    def control_loop(self):

        armed = self.get_parameter(
            'armed'
        ).value

        # ============================================================
        # DESARMADO
        # ============================================================

        if not armed:

            self.publish_stop()

            self.was_armed = False

            self.state = 'IDLE'

            return

        # ============================================================
        # SEM ODOMETRIA
        # ============================================================

        if not self.have_odom:

            self.publish_stop()

            self.get_logger().warn(
                'Aguardando odometria...'
            )

            return

        # ============================================================
        # FOI ARMADO AGORA
        # ============================================================

        if not self.was_armed:

            if not self.initialize_goal():

                self.publish_stop()

                return

            self.was_armed = True

            self.previous_time = (
                self.get_clock().now()
            )

        # ============================================================
        # META JÁ ATINGIDA
        # ============================================================

        if self.goal_reached:

            self.publish_stop()

            return

        # ============================================================
        # TEMPO
        # ============================================================

        current_time = self.get_clock().now()

        dt = (
            current_time
            - self.previous_time
        ).nanoseconds / 1e9

        self.previous_time = current_time

        if dt <= 0.0:
            return

        # ============================================================
        # ERRO DE POSIÇÃO
        # ============================================================

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance_error = math.sqrt(
            dx * dx + dy * dy
        )

        # ============================================================
        # ERRO ANGULAR PARA A POSIÇÃO
        # ============================================================

        target_angle = math.atan2(
            dy,
            dx
        )

        heading_error = self.normalize_angle(
            target_angle - self.theta
        )

        # ============================================================
        # ERRO ANGULAR FINAL
        # ============================================================

        final_angle_error = self.normalize_angle(
            self.goal_theta - self.theta
        )

        # ============================================================
        # TOLERÂNCIAS
        # ============================================================

        position_tolerance = self.get_parameter(
            'position_tolerance'
        ).value

        angle_tolerance = self.get_parameter(
            'angle_tolerance'
        ).value

        # ============================================================
        # FASE 1 - ANDAR
        # ============================================================

        if self.state == 'MOVE':

            # --------------------------------------------------------
            # Chegou à posição
            # --------------------------------------------------------

            if distance_error <= position_tolerance:

                self.publish_stop()

                # Reset PID angular
                self.integral_angular = 0.0
                self.previous_angular_error = 0.0

                self.state = 'ROTATE'

                self.get_logger().info(
                    '----------------------------------------'
                )

                self.get_logger().info(
                    'DISTÂNCIA ATINGIDA!'
                )

                self.get_logger().info(
                    f'Erro de posição: '
                    f'{distance_error:.3f} m'
                )

                self.get_logger().info(
                    'FASE 2: GIRAR'
                )

                self.get_logger().info(
                    f'Ângulo alvo: '
                    f'{math.degrees(self.goal_theta):.2f}°'
                )

                self.get_logger().info(
                    '----------------------------------------'
                )

                return

            # --------------------------------------------------------
            # PID LINEAR
            # --------------------------------------------------------

            kp_linear = self.get_parameter(
                'kp_linear'
            ).value

            ki_linear = self.get_parameter(
                'ki_linear'
            ).value

            kd_linear = self.get_parameter(
                'kd_linear'
            ).value

            self.integral_linear += (
                distance_error * dt
            )

            integral_limit_linear = self.get_parameter(
                'integral_limit_linear'
            ).value

            self.integral_linear = max(
                -integral_limit_linear,
                min(
                    self.integral_linear,
                    integral_limit_linear
                )
            )

            derivative_linear = (
                distance_error
                - self.previous_linear_error
            ) / dt

            linear_velocity = (
                kp_linear * distance_error
                + ki_linear * self.integral_linear
                + kd_linear * derivative_linear
            )

            self.previous_linear_error = (
                distance_error
            )

            # --------------------------------------------------------
            # PID ANGULAR
            #
            # Durante a fase de movimento usamos o ângulo para
            # manter o robô apontado para o objetivo.
            # --------------------------------------------------------

            kp_angular = self.get_parameter(
                'kp_angular'
            ).value

            ki_angular = self.get_parameter(
                'ki_angular'
            ).value

            kd_angular = self.get_parameter(
                'kd_angular'
            ).value

            self.integral_angular += (
                heading_error * dt
            )

            integral_limit_angular = self.get_parameter(
                'integral_limit_angular'
            ).value

            self.integral_angular = max(
                -integral_limit_angular,
                min(
                    self.integral_angular,
                    integral_limit_angular
                )
            )

            derivative_angular = (
                heading_error
                - self.previous_angular_error
            ) / dt

            angular_velocity = (
                kp_angular * heading_error
                + ki_angular * self.integral_angular
                + kd_angular * derivative_angular
            )

            self.previous_angular_error = (
                heading_error
            )

            # --------------------------------------------------------
            # SEGURANÇA
            #
            # Se estiver muito desalinhado, não anda.
            # --------------------------------------------------------

            max_heading_for_motion = 0.30

            if abs(heading_error) > max_heading_for_motion:

                linear_velocity = 0.0

            # ========================================================
            # SATURAÇÃO
            # ========================================================

            max_linear_velocity = self.get_parameter(
                'max_linear_velocity'
            ).value

            max_angular_velocity = self.get_parameter(
                'max_angular_velocity'
            ).value

            linear_velocity = max(
                -max_linear_velocity,
                min(
                    linear_velocity,
                    max_linear_velocity
                )
            )

            angular_velocity = max(
                -max_angular_velocity,
                min(
                    angular_velocity,
                    max_angular_velocity
                )
            )

            # ========================================================
            # PUBLICA
            # ========================================================

            cmd = Twist()

            cmd.linear.x = linear_velocity

            cmd.linear.y = 0.0
            cmd.linear.z = 0.0

            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = angular_velocity

            self.cmd_pub.publish(cmd)

            self.write_log(
                distance_error,
                heading_error,
                linear_velocity,
                angular_velocity
            )

            return

        # ============================================================
        # FASE 2 - GIRAR
        # ============================================================

        if self.state == 'ROTATE':

            # --------------------------------------------------------
            # Chegou ao ângulo
            # --------------------------------------------------------

            if abs(final_angle_error) <= angle_tolerance:

                self.publish_stop()

                self.state = 'DONE'
                self.goal_reached = True

                self.get_logger().info(
                    '========================================'
                )

                self.get_logger().info(
                    'MOVIMENTO COMPLETO!'
                )

                self.get_logger().info(
                    f'Erro de posição: '
                    f'{distance_error:.3f} m'
                )

                self.get_logger().info(
                    f'Erro angular: '
                    f'{final_angle_error:.4f} rad '
                    f'({math.degrees(final_angle_error):.2f}°)'
                )

                self.get_logger().info(
                    f'Pose final: '
                    f'x={self.x:.3f} '
                    f'y={self.y:.3f} '
                    f'theta={math.degrees(self.theta):.2f}°'
                )

                self.get_logger().info(
                    'ROBÔ PARADO.'
                )

                self.get_logger().info(
                    '========================================'
                )

                self.write_log(
                    distance_error,
                    final_angle_error,
                    0.0,
                    0.0
                )

                return

            # --------------------------------------------------------
            # PID ANGULAR
            # --------------------------------------------------------

            kp_angular = self.get_parameter(
                'kp_angular'
            ).value

            ki_angular = self.get_parameter(
                'ki_angular'
            ).value

            kd_angular = self.get_parameter(
                'kd_angular'
            ).value

            self.integral_angular += (
                final_angle_error * dt
            )

            integral_limit_angular = self.get_parameter(
                'integral_limit_angular'
            ).value

            self.integral_angular = max(
                -integral_limit_angular,
                min(
                    self.integral_angular,
                    integral_limit_angular
                )
            )

            derivative_angular = (
                final_angle_error
                - self.previous_angular_error
            ) / dt

            angular_velocity = (
                kp_angular * final_angle_error
                + ki_angular * self.integral_angular
                + kd_angular * derivative_angular
            )

            self.previous_angular_error = (
                final_angle_error
            )

            # --------------------------------------------------------
            # DURANTE A ROTAÇÃO NÃO ANDA
            # --------------------------------------------------------

            linear_velocity = 0.0

            # ========================================================
            # SATURAÇÃO
            # ========================================================

            max_angular_velocity = self.get_parameter(
                'max_angular_velocity'
            ).value

            angular_velocity = max(
                -max_angular_velocity,
                min(
                    angular_velocity,
                    max_angular_velocity
                )
            )

            # ========================================================
            # PUBLICA
            # ========================================================

            cmd = Twist()

            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0

            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = angular_velocity

            self.cmd_pub.publish(cmd)

            self.write_log(
                distance_error,
                final_angle_error,
                0.0,
                angular_velocity
            )

            return

        # ============================================================
        # FASE 3 - FINALIZADO
        # ============================================================

        if self.state == 'DONE':

            self.publish_stop()

            return

    # ================================================================
    # LOG
    # ================================================================

    def write_log(
        self,
        position_error,
        angle_error,
        linear_velocity,
        angular_velocity
    ):

        if self.experiment_start_time is None:
            return

        elapsed_time = (
            self.get_clock().now()
            - self.experiment_start_time
        ).nanoseconds / 1e9

        self.csv_writer.writerow([
            elapsed_time,
            self.state,
            self.x,
            self.y,
            self.theta,
            self.goal_x,
            self.goal_y,
            self.goal_theta,
            position_error,
            angle_error,
            linear_velocity,
            angular_velocity
        ])

        self.log_file.flush()

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self):

        self.publish_stop()

        if self.log_file is not None:

            self.log_file.flush()
            self.log_file.close()

            self.log_file = None

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = PIDPositionController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.publish_stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()