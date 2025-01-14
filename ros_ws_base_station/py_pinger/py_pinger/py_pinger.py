
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemSend, ModemRec, ModemCmdUpdate
from .seatrac_enums import CID_E, AMSGTYPE_E, CST_E
from time import sleep

class SeatracPinger(Node):

    def __init__(self):
        super().__init__('pinger')

        self.declare_parameter("target_id", 15)

        self.target_id = self.get_parameter("target_id").get_parameter_value().integer_value

        self.modem_publisher_  = self.create_publisher(ModemSend, 'modem_send', 10)
        self.modem_rec_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_rec_callback, 10)
        self.modem_cmd_update_subscriber_ = self.create_subscription(
            ModemRec, 'modem_cmd_update', self.modem_cmd_update_callback, 10)

        self.send_ping()


    def send_ping(self):
        sleep(1)
        request = ModemSend()
        request.msg_id      = CID_E.CID_PING_SEND
        request.dest_id     = self.target_id
        request.msg_type    = AMSGTYPE_E.MSG_REQU
        self.modem_publisher_.publish(request)


    def modem_rec_callback(self, response):
        if response.msg_id == CID_E.CID_PING_RESP:
            self.send_ping()

    def modem_cmd_update_callback(self, response):
        if response.msg_id == CID_E.CID_PING_ERROR:
            self.send_ping()
            self.get_logger().error(f"Seatrac Ping Error. Target Beacon Id: {response.target_id}. Error Code: {CST_E.to_str(response.command_status_code)}")
        if response.msg_id == CID_E.CID_PING_SEND and not response.command_status_code == CST_E.CST_OK:
            self.send_ping()
            self.get_logger().error(f"Seatrac Ping Send Error. Target Beacon Id: {response.target_id}. Error Code: {CST_E.to_str(response.command_status_code)}")

                


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SeatracPinger())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
