# from base_station_interfaces.srv import BeaconId
# import random 
# import rclpy
# from rclpy.node import Node


# class EKillService(Node):

#     def __init__(self):
#         super().__init__('e_kill_service_node')
#         self.srv = self.create_service(BeaconId, 'e_kill_service', self.e_kill_callback)
#         self.srv_surface = self.create_service(BeaconId, 'e_surface_service', self.e_surface_callback)

#     def e_kill_callback(self, request, response):
#         # Log the request (customize as needed)
#         self.get_logger().info(f"Received e_kill_service request: {request}")
#         response.success = random.choice([True, False])
#         return response

#     def e_surface_callback(self, request, response):
#         # Log the request (customize as needed)
#         response.success = random.choice([True, False])
#         self.get_logger().info(f"Received e_surface_service request: {request}")
#         return response

# def main():
#     rclpy.init()
#     node = EKillService()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()