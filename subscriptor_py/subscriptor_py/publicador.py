import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading
import time 
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32, 'diferencial_control_remoto', 10)

    def publish_message(self, data):
        msg = Int32()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    def spin_thread():
        rclpy.spin(minimal_publisher)

    thread = threading.Thread(target=spin_thread)
    thread.start()

    try:
        while True:
            command = input("0: Detener, 1: Frente, 2: Retroceso, 3: Izquierda, 4: Derecha, 5: Secuencia, 6: break = ")
            if command == "5":
                minimal_publisher.publish_message(1)
                time.sleep(4)
                minimal_publisher.publish_message(2)
                time.sleep(4)
                minimal_publisher.publish_message(3)
                time.sleep(4)
                minimal_publisher.publish_message(4)
                time.sleep(4)
                minimal_publisher.publish_message(0)
                time.sleep(4)
            elif command == "0":
                minimal_publisher.publish_message(0)
            elif command == "1":
                minimal_publisher.publish_message(1)
            elif command == "2":
                minimal_publisher.publish_message(2)
            elif command == "3":
                minimal_publisher.publish_message(3)
            elif command == "4":
                minimal_publisher.publish_message(4)
            elif command == "6":
                break
            else:
                print("Invalid command. Please enter 0, 1,3, 4, 5 or 6.")
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == '_main_':
    main()
