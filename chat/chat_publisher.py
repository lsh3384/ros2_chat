import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import argparse


class ChatPublisher(Node):
    def __init__(self, nickname):
        super().__init__('chat')
        qos_profile = QoSProfile(depth=10)
        self.chat_subscriber = self.create_subscription(
            String,
            'enjoychat',
            self.subscribe_topic_message,
            qos_profile)

        qos_profile = QoSProfile(depth=10)
        self.chat_publisher = self.create_publisher(String, 'enjoychat', qos_profile)
        self.publish_chat(nickname)

    def subscribe_topic_message(self, msg):
        print(msg.data)

    def publish_chat(self, nickname):


        def chat_input():
            time.sleep(0.1)
            msg = String()
            msg.data = nickname + "님이 입장하셨습니다."
            self.chat_publisher.publish(msg)
            while(True):
                msg = String()
                data = input()
                print ("\033[A\033[A")
                msg.data = f'{nickname} : {str(data)}'
                self.chat_publisher.publish(msg)
                if data == 'exit':
                    msg.data = f'{nickname}님이 채팅방에서 나가셨습니다.'
                    self.chat_publisher.publish(msg)
                    break

        t = threading.Thread(target=chat_input)
        t.start()


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-nick',
        '--nickname',
        type=str,
        default=50,
        help='nickname of the user')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args()

    rclpy.init(args=args.argv)
    node = ChatPublisher(args.nickname)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        # rclpy.spin(node)
        executor.spin()
        node.chat_publisher.publish('hello')
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
