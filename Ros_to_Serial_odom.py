import rclpy
import serial
import struct
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


# ================= CONFIG =================

PORT = '/dev/ttyUSB1'
BAUDRATE = 115200

START_HEADER = 0x39
END_HEADER = 0x55
FRAME_SIZE = 8


# ================= CRC-8 =================

def crc8(data: bytes) -> int:

    crc = 0x00

    for byte in data:

        crc ^= byte

        for _ in range(8):

            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


# ================= BUILD FRAME =================

def build_frame(msg_id: int, payload: bytes) -> bytes:

    frame = bytearray(8)

    frame[0] = START_HEADER
    frame[1] = msg_id
    frame[2:6] = payload
    frame[6] = crc8(frame[1:6])
    frame[7] = END_HEADER

    return frame


# ================= PARSE FRAME =================

def parse_frame(frame: bytes):

    if len(frame) != FRAME_SIZE:
        return None

    if frame[0] != START_HEADER:
        return None

    if frame[7] != END_HEADER:
        return None

    if crc8(frame[1:6]) != frame[6]:
        return None

    msg_id = frame[1]
    data = frame[2:6]

    return msg_id, data


# ================= ROS NODE =================

class RosSerial(Node):
    def __init__(self):
        super().__init__('ros_to_serial')

        # ---------- SERIAL ----------

        try:

            self.ser_handler = serial.Serial(PORT,BAUDRATE,timeout=0.01)
            self.get_logger().info(
                f'Serial aberta: {PORT} @ {BAUDRATE}'
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Erro ao abrir serial: {e}'
            )

            raise


        # ---------- ROS CMD_VEL ----------

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )


        # ---------- ODOMETRIA ----------

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        id_msg = 0x0D
        
        payload = [0,0,0,5]
        tx_frame = build_frame(id_msg, payload)
    
        self.ser_handler.write(tx_frame)
    
        print(tx_frame)


        # ---------- TF ----------

        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- TF Estática -------------

        # self.static_tf_broadcaster = StaticTransformBroadcaster(self)


        # ---------- ESTADO DO ROBÔ ----------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


        # ---------- BUFFER SERIAL ----------

        self.rx_buffer = bytearray()


        # ---------- TIMER ----------

        # Executa a leitura da serial a 100 Hz
        self.serial_timer = self.create_timer(
            0.01,
            self.read_serial
        )


        # Publica odometria a 50 Hz
        self.odom_timer = self.create_timer(
            0.02,
            self.publish_odometry
        )


        self.get_logger().info(
            'Bridge ROS ↔ Serial ativo'
        )


    # ==================================================
    # RECEBE /cmd_vel E ENVIA PARA RP2040
    # ==================================================

    def cmd_vel_callback(self, msg):

        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        ID_MSG = 0x04

        payload = bytearray(4)

        escala = 1000


        # Linear -> int16
        linear_int = int(linear_speed * escala)

        payload[0:2] = linear_int.to_bytes(
            2,
            byteorder='little',
            signed=True
        )


        # Angular -> int16
        angular_int = int(angular_speed * escala)

        payload[2:4] = angular_int.to_bytes(
            2,
            byteorder='little',
            signed=True
        )


        tx_frame = build_frame(
            ID_MSG,
            payload
        )


        try:

            self.ser_handler.write(tx_frame)

        except serial.SerialException as e:

            self.get_logger().error(
                f'Erro ao escrever na serial: {e}'
            )


    # ==================================================
    # LEITURA DA SERIAL
    # ==================================================

    def read_serial(self):
        try:
            if self.ser_handler.in_waiting:
                # Lê todos os bytes disponíveis
                self.rx_buffer.extend(
			    self.ser_handler.read(self.ser_handler.in_waiting)
		        )
                print("Há mensgens disponíveis")


            # Enquanto houver pelo menos um frame
            while len(self.rx_buffer) >= FRAME_SIZE:


                # ------------------------------------------
                # SINCRONIZA COM START_HEADER
                # ------------------------------------------

                if self.rx_buffer[0] != START_HEADER:

                    self.rx_buffer.pop(0)

                    continue

                # ------------------------------------------
                # PEGA FRAME
                # ------------------------------------------

                frame = bytes(
                    self.rx_buffer[:FRAME_SIZE]
                )


                # ------------------------------------------
                # PROCESSA FRAME
                # ------------------------------------------

                result = parse_frame(frame)

                if result:
                    msg_id, data = result

                    print(
                        f"RX -> ID: 0x{msg_id:02X}, "
                        f"bytes: {[hex(b) for b in data]}"
                    )

                    if msg_id == 0x10:
                        value = struct.unpack('<f', data)[0]
                        print(f"ODOM X = {value}")

                    elif msg_id == 0x11:
                        value = struct.unpack('<f', data)[0]
                        print(f"ODOM Y = {value}")

                    elif msg_id == 0x12:
                        value = struct.unpack('<f', data)[0]
                        print(f"ODOM THETA = {value}")


                if result:

                    msg_id, data = result


                    # Os dados X/Y/THETA são float32
                    value_rx = struct.unpack(
                        '<f',
                        data
                    )[0]


                    # --------------------------------------
                    # ODOMETRIA
                    # --------------------------------------

                    if msg_id == 0x10:

                        self.x = value_rx

                        # Descomente para debug
                        # self.get_logger().info(
                        #     f'x = {self.x:.3f}'
                        # )


                    elif msg_id == 0x11:

                        self.y = value_rx

                        # self.get_logger().info(
                        #     f'y = {self.y:.3f}'
                        # )


                    elif msg_id == 0x12:

                        self.theta = value_rx

                        # self.get_logger().info(
                        #     f'theta = {self.theta:.3f}'
                        # )


                    # --------------------------------------
                    # VELOCIDADES DOS MOTORES
                    # --------------------------------------

                    elif msg_id == 0x09:

                        self.get_logger().debug(
                            f'M1 = {value_rx:.3f}'
                        )


                    elif msg_id == 0x0A:

                        self.get_logger().debug(
                            f'M2 = {value_rx:.3f}'
                        )


                    elif msg_id == 0x0B:

                        self.get_logger().debug(
                            f'M3 = {value_rx:.3f}'
                        )


                    elif msg_id == 0x0C:

                        self.get_logger().debug(
                            f'M4 = {value_rx:.3f}'
                        )


                    # Remove o frame processado
                    del self.rx_buffer[:FRAME_SIZE]


                else:

                    # Frame inválido.
                    # Remove um byte e tenta sincronizar novamente.
                    self.rx_buffer.pop(0)

        except serial.SerialException as e:
            self.get_logger().error(
                f'Erro ao escrever na serial: {e}'
            )




    # ==================================================
    # PUBLICA /odom
    # ==================================================

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()


        # ---------- QUATERNION ----------

        # theta é rotação em torno do eixo Z
        qz = __import__('math').sin(self.theta / 2.0)
        qw = __import__('math').cos(self.theta / 2.0)


        # ---------- ODOM MESSAGE ----------

        odom_msg = Odometry()

        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'

        odom_msg.child_frame_id = 'base_link'


        # Posição

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0


        # Orientação

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw



        # Publica

        self.odom_pub.publish(
            odom_msg
        )


        # ==================================================
        # PUBLICA TF: odom -> base_link
        # ==================================================

        transform = TransformStamped()

        transform.header.stamp = now

        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'


        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0


        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw


        self.tf_broadcaster.sendTransform(
            transform
        )

        # # ==================================================
        # # TF ESTÁTICA: base_link -> laser
        # # ==================================================

        # laser_tf = TransformStamped()

        # laser_tf.header.stamp = self.get_clock().now().to_msg()
        # laser_tf.header.frame_id = 'base_link'
        # laser_tf.child_frame_id = 'laser'

        # # Posição do Lidar em relação ao base_link
        # laser_tf.transform.translation.x = 0.0
        # laser_tf.transform.translation.y = 0.0
        # laser_tf.transform.translation.z = 0.13

        # # Orientação do Lidar
        # laser_tf.transform.rotation.x = 0.0
        # laser_tf.transform.rotation.y = 0.0
        # laser_tf.transform.rotation.z = 0.0
        # laser_tf.transform.rotation.w = 1.0

        # self.static_tf_broadcaster.sendTransform(
        #     laser_tf
        # )



    # ==================================================
    # ENCERRAMENTO
    # ==================================================

    def destroy_node(self):

        if hasattr(self, 'ser_handler'):

            if self.ser_handler.is_open:

                self.ser_handler.close()

                print('Serial fechada')

        super().destroy_node()


# ================= MAIN =================

def main():
    rclpy.init()
    node = RosSerial()
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Encerrando bridge...')

    finally:
        if node.ser_handler is not None and node.ser_handler.is_open:
            node.ser_handler.close()
            print('Serial fechada')

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':

    main()
