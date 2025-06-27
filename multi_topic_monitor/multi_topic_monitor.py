import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
import yaml
import importlib

class MultiTopicMonitor(Node):
    def __init__(self, config_path):
        super().__init__('multi_topic_monitor')

        self.timeout_seconds = 5.0
        self.last_received = {}

        # Load config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.topics_config = config.get('topics', {})
        self.topic_types = {}
        self.last_received = {topic: 0 for topic in self.topics_config}

        # Set up subscriptions
        for topic, type_str in self.topics_config.items():
            msg_class = self.import_message_class(type_str)
            if msg_class is None:
                self.get_logger().error(f"Could not import message type: {type_str}")
                continue
            self.topic_types[topic] = msg_class
            self.create_subscription(
                msg_class,
                topic,
                self.create_callback(topic),
                10
            )
            self.get_logger().info(f"Subscribed to {topic} as {type_str}")

        # Status publisher
        self.status_pub = self.create_publisher(String, '/multi_topic_monitor', 10)
        self.create_timer(5.0, self.check_and_publish_status)

    def import_message_class(self, type_str):
        try:
            pkg, msg_module, msg_name = type_str.split('/')
            module = importlib.import_module(f"{pkg}.{msg_module}")
            return getattr(module, msg_name)
        except Exception as e:
            self.get_logger().error(f"Failed to import {type_str}: {e}")
            return None

    def create_callback(self, topic_name):
        def callback(msg):
            self.last_received[topic_name] = time.time()
        return callback

    def check_and_publish_status(self):
        now = time.time()
        status_dict = {}

        for topic in self.topics_config:
            elapsed = now - self.last_received[topic]
            status_dict[topic] = 1 if elapsed < self.timeout_seconds else 0

        msg = String()
        msg.data = json.dumps(status_dict)
        self.status_pub.publish(msg)

        self.get_logger().info(f"Published JSON status: {msg.data}")

def main(args=None):
    import sys
    if len(sys.argv) < 2:
        print("Usage: ros2 run your_package multi_topic_monitor.py /path/to/config.yaml")
        return

    config_path = sys.argv[1]

    rclpy.init(args=args)
    node = MultiTopicMonitor(config_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
