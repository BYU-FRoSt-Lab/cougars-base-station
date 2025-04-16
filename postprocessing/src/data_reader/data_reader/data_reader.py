
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemRec
from .seatrac_utils import CID_E
from wamv_interfaces.msg import MicrostrainGnss

class SeatracLogger(Node):

    def __init__(self):
        super().__init__('logger')
        self.modem_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_callback, 10)
        self.wamv_subscriber_  = self.create_subscription(MicrostrainGnss, '/micro_gnss', self.gps_callback, 10)

        if not os.path.exists("csv_data"):
            os.mkdir("csv_data")

        time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        self.modem_data = open(f"csv_data/modem_data.csv", 'w')
        self.modem_data.write(
            "timestamp, remote_beacon_id, position_flt_error, yaw, pitch, roll, local_depth, VOS, RSSI, usbl_rssi[0], usbl_rssi[1], usbl_rssi[2], usbl_rssi[3], range_time, range_dist, azimuth, elevation, fit_error, easting, northing, remote_depth\n")

        self.modem_err_data = open(f"csv_data/modem_error_data.csv", 'w')
        self.modem_err_data.write("timestamp, cst_code, remote_beacon_id\n")

        self.gps_data = open(f"csv_data/gps_data.csv", 'w')
        self.gps_data.write("timestamp, latitude, longitude, heading\n")
    

        self.line_num = 0


    def modem_callback(self, response):
        if(response.msg_id == CID_E.CID_PING_RESP):
            csv_line = (
                str(response.system_timestamp)    +", "+
                str(response.src_id)              +", "+
                str(int(response.position_flt_error))  +", "+
                str(response.attitude_yaw)        +", "+
                str(response.attitude_pitch)      +", "+
                str(response.attitude_roll)       +", "+
                str(response.depth_local)         +", "+
                str(response.vos)                 +", "+
                str(response.rssi)                +", "+
                str(response.usbl_rssi[0])        +", "+
                str(response.usbl_rssi[1])        +", "+
                str(response.usbl_rssi[2])        +", "+
                str(response.usbl_rssi[3])        +", "+
                str(response.range_time)          +", "+
                str(response.range_dist)          +", "+
                str(response.usbl_azimuth)        +", "+
                str(response.usbl_elevation)      +", "+
                str(response.usbl_fit_error)      +", "+
                str(response.position_easting)    +", "+
                str(response.position_northing)   +", "+
                str(response.position_depth)      +"\n"
            )
            self.modem_data.write(csv_line)
        if(response.msg_id == CID_E.CID_PING_ERROR):
            csv_line = (
                str(response.system_timestamp)    +", "+
                str(response.command_status_code) +", "+
                str(response.target_id)           +"\n"
            )
            self.modem_err_data.write(csv_line)


    def gps_callback(self, response):
        csv_line = (
            str(response.timestamp)         +", "+
            str(response.latitude)          +", "+
            str(response.longitude)         +", "+
            str(response.heading)           +"\n"
        )
        self.gps_data.write(csv_line)


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracLogger()
    rclpy.spin(seatrac_logger)
    seatrac_logger.modem_data.close()
    seatrac_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()