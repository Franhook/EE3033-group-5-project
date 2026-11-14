import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Important: import the service type defined in the balance_interface package
from balance_interface.srv import SetTarget

class ObjectDetectClient(Node):
    def __init__(self):
        super().__init__('test_object_detect_client')
        # Create a client connected to the '/detector/set_target' service
        self.cli = self.create_client(SetTarget, '/detector/set_target')
        self.bridge = CvBridge()

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service /detector/set_target ...')
        
    def send_request(self, image_path):
        # 1. Read a local image
        if not os.path.exists(image_path):
            self.get_logger().error(f"File not found: {image_path}")
            return False

        cv_img = cv2.imread(image_path)
        if cv_img is None:
            self.get_logger().error(f"Failed to decode image: {image_path}")
            return False

        self.get_logger().info(f"Sending image: {image_path} ({cv_img.shape})")

        # 2. Construct the request
        req = SetTarget.Request()
        # Convert the OpenCV image to a ROS message
        req.target_image = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")

        # 3. Send the request asynchronously
        future = self.cli.call_async(req)
        
        # 4. Wait for the result
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    # Basic parameter validation
    if len(sys.argv) < 2:
        print("Usage: ros2 run balance_object_detect test_object_detect_client <path_to_image>")
        return

    image_path = sys.argv[1]
    
    client = ObjectDetectClient()
    response = client.send_request(image_path)

    if response:
        if response.success:
            client.get_logger().info(f"SUCCESS: {response.message}")
        else:
            client.get_logger().warn(f"FAILED: {response.message}")
    else:
        client.get_logger().error("Service call failed.")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
