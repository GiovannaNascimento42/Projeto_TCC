import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PlantIdentifier(Node):
    def __init__(self):
        super().__init__('plant_identifier')

        self.declare_parameter('armed', False)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('linear_step', 0.08)
        self.declare_parameter('linear_duration', 5.0)
        self.declare_parameter('linear_recovery', 3.0)
        self.declare_parameter('between_time', 2.0)
        self.declare_parameter('angular_step', 0.20)
        self.declare_parameter('angular_duration', 5.0)
        self.declare_parameter('angular_recovery', 3.0)
        self.declare_parameter('output_dir', os.path.expanduser('~/plant_experiments'))

        self.armed = bool(self.get_parameter('armed').value)
        self.freq = float(self.get_parameter('control_frequency').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.linear_duration = float(self.get_parameter('linear_duration').value)
        self.linear_recovery = float(self.get_parameter('linear_recovery').value)
        self.between_time = float(self.get_parameter('between_time').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.angular_duration = float(self.get_parameter('angular_duration').value)
        self.angular_recovery = float(self.get_parameter('angular_recovery').value)
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 20
        )

        self.x = self.y = self.theta = None
        self.v_pose = 0.0
        self.w_pose = 0.0
        self.last_odom_time = None
        self.prev_x = self.prev_y = self.prev_theta = None

        self.phase = 'IDLE'
        self.phase_start = None
        self.experiment_start = None
        self.csv_file = None
        self.csv_writer = None
        self.rows = []

        self.last_armed = self.armed
        if self.armed:
            self.get_logger().warn(
                'armed=true ao iniciar. O experimento comecará quando houver /odom.'
            )
        else:
            self.get_logger().warn(
                'Plant Identifier: armed=false. NAO rode este no junto com o PID.'
            )

        self.timer = self.create_timer(1.0 / self.freq, self.control_loop)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # yaw de quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.x is not None and self.last_odom_time is not None:
            dt = stamp - self.last_odom_time
            if 1e-4 < dt < 0.5:
                dx = p.x - self.x
                dy = p.y - self.y
                self.v_pose = math.hypot(dx, dy) / dt
                dtheta = wrap_angle(theta - self.theta)
                self.w_pose = dtheta / dt

        self.x, self.y, self.theta = p.x, p.y, theta
        self.last_odom_time = stamp

    def cmd(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        if rclpy.ok():
            self.cmd_pub.publish(m)

    def start_experiment(self):
        if self.x is None:
            return

        os.makedirs(self.output_dir, exist_ok=True)
        filename = os.path.join(
            self.output_dir,
            'plant_experiment_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
        )
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'phase', 'cmd_linear', 'cmd_angular',
            'x', 'y', 'theta', 'v_pose', 'w_pose'
        ])

        self.rows = []
        self.experiment_start = self.get_clock().now()
        self.phase_start = self.experiment_start
        self.phase = 'PRETEST'
        self.get_logger().info('FASE: PRETEST')
        self.get_logger().warn('Experimento iniciado; zero por 2.0 s.')
        self.get_logger().info(f'CSV: {filename}')

    def elapsed_phase(self):
        return (self.get_clock().now() - self.phase_start).nanoseconds * 1e-9

    def elapsed_total(self):
        return (self.get_clock().now() - self.experiment_start).nanoseconds * 1e-9

    def set_phase(self, phase):
        self.phase = phase
        self.phase_start = self.get_clock().now()
        self.get_logger().info(f'FASE: {phase}')

    def log_sample(self, vcmd, wcmd):
        if self.csv_writer is None or self.x is None:
            return
        row = [
            self.elapsed_total(),
            self.phase,
            vcmd, wcmd,
            self.x, self.y, self.theta,
            self.v_pose, self.w_pose
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.rows.append(row)

    def control_loop(self):
        # Detecta alteração do parâmetro armed sem precisar reiniciar o nó.
        armed = bool(self.get_parameter('armed').value)

        if not armed:
            self.phase = 'IDLE'
            self.cmd(0.0, 0.0)
            self.last_armed = False
            return

        if not self.last_armed:
            if self.x is None:
                return
            self.last_armed = True
            self.start_experiment()
            return

        if self.phase == 'IDLE':
            return

        t = self.elapsed_phase()
        vcmd = 0.0
        wcmd = 0.0

        if self.phase == 'PRETEST':
            if t >= 2.0:
                self.set_phase('LINEAR_STEP')
                t = 0.0
            else:
                vcmd = 0.0

        elif self.phase == 'LINEAR_STEP':
            vcmd = self.linear_step
            if t >= self.linear_duration:
                self.set_phase('LINEAR_RECOVERY')

        elif self.phase == 'LINEAR_RECOVERY':
            if t >= self.linear_recovery:
                self.set_phase('BETWEEN')

        elif self.phase == 'BETWEEN':
            if t >= self.between_time:
                self.set_phase('ANGULAR_STEP')
                t = 0.0

        elif self.phase == 'ANGULAR_STEP':
            wcmd = self.angular_step
            if t >= self.angular_duration:
                self.set_phase('ANGULAR_RECOVERY')

        elif self.phase == 'ANGULAR_RECOVERY':
            if t >= self.angular_recovery:
                self.set_phase('FINISHED')
                self.cmd(0.0, 0.0)
                self.finish_experiment()
                return

        elif self.phase == 'FINISHED':
            self.cmd(0.0, 0.0)
            return

        self.cmd(vcmd, wcmd)
        self.log_sample(vcmd, wcmd)

    def estimate_first_order(self, phase, cmd_col, out_prefix):
        rows = [r for r in self.rows if r[1] == phase]
        if len(rows) < 10:
            return None

        t = [float(r[0]) for r in rows]
        y = [float(r[7] if cmd_col == 'linear' else r[8]) for r in rows]
        u = [float(r[2] if cmd_col == 'linear' else r[3]) for r in rows]

        u_step = self.linear_step if cmd_col == 'linear' else self.angular_step
        y0 = y[0]

        # média dos últimos 20% como regime permanente
        n_tail = max(5, len(y) // 5)
        yss = sum(y[-n_tail:]) / n_tail

        dy = yss - y0
        if abs(u_step) < 1e-9 or abs(dy) < 1e-6:
            return None

        K = dy / u_step

        # Valores alvo para identificar L e tau.
        # 5%: início efetivo da resposta.
        # 63.2%: definição clássica de tau para 1ª ordem.
        sign = 1.0 if dy >= 0 else -1.0
        y5 = y0 + 0.05 * dy
        y632 = y0 + 0.632 * dy

        i5 = None
        i632 = None
        for i, yy in enumerate(y):
            if sign * (yy - y5) >= 0:
                i5 = i
                break
        for i, yy in enumerate(y):
            if sign * (yy - y632) >= 0:
                i632 = i
                break

        if i632 is None:
            return None

        # Interpolação simples dos tempos de cruzamento.
        def crossing_time(target, i):
            if i == 0:
                return t[0]
            y1, y2 = y[i-1], y[i]
            t1, t2 = t[i-1], t[i]
            if abs(y2-y1) < 1e-12:
                return t2
            a = (target-y1)/(y2-y1)
            a = max(0.0, min(1.0, a))
            return t1 + a*(t2-t1)

        t632 = crossing_time(y632, i632)

        if i5 is not None:
            t5 = crossing_time(y5, i5)
        else:
            t5 = t[0]

        # Para uma resposta de 1ª ordem sem atraso, L aproximado
        # pelo início de 5% e tau pelo intervalo até 63.2%.
        L = max(0.0, t5 - t[0])
        tau = max(0.0, t632 - t5)

        # Se L ficou praticamente zero, isso indica resposta muito rápida
        # em relação à amostragem e não deve ser tratado como um atraso negativo.
        return {
            'K': K,
            'L': L,
            'tau': tau,
            'y0': y0,
            'yss': yss,
            'u_step': u_step,
            't5': t5,
            't632': t632,
            'n': len(rows)
        }

    def write_estimate(self, name, result):
        if result is None:
            self.get_logger().warn(f'Nao foi possivel estimar {name}')
            return

        filename = os.path.join(
            self.output_dir,
            f'plant_estimate_{name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.txt'
        )
        with open(filename, 'w') as f:
            f.write(f'Tipo: {name}\n')
            f.write(f"step: {result['u_step']:.8f}\n")
            f.write(f"K: {result['K']:.8f}\n")
            f.write(f"L: {result['L']:.8f} s\n")
            f.write(f"tau: {result['tau']:.8f} s\n")
            f.write(f"y0: {result['y0']:.8f}\n")
            f.write(f"yss: {result['yss']:.8f}\n")
            f.write(f"t5: {result['t5']:.8f} s\n")
            f.write(f"t632: {result['t632']:.8f} s\n")
            f.write(f"amostras: {result['n']}\n")
            f.write('\nModelo aproximado:\n')
            f.write(
                f"G(s) = {result['K']:.8f} * exp(-{result['L']:.8f}s) / "
                f"({result['tau']:.8f}s + 1)\n"
            )
        self.get_logger().info(f'Estimativa {name}: {filename}')
        self.get_logger().info(
            f"K={result['K']:.5f}, L={result['L']:.5f}s, tau={result['tau']:.5f}s"
        )

    def finish_experiment(self):
        self.cmd(0.0, 0.0)
        linear = self.estimate_first_order('LINEAR_STEP', 'linear', 'linear')
        angular = self.estimate_first_order('ANGULAR_STEP', 'angular', 'angular')
        self.write_estimate('linear', linear)
        self.write_estimate('angular', angular)

        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None

        self.phase = 'FINISHED'
        self.get_logger().info('Experimento concluído. armed será mantido em false.')

        # Evita reinício automático.
        self.set_parameters([rclpy.parameter.Parameter(
            'armed',
            rclpy.Parameter.Type.BOOL,
            False
        )])
        self.last_armed = False

    def close(self):
        # Só publica se o contexto ROS ainda estiver válido.
        if rclpy.ok():
            try:
                self.cmd(0.0, 0.0)
            except Exception:
                pass
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PlantIdentifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
