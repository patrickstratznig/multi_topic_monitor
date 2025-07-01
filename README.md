# `multi_topic_monitor`

`multi_topic_monitor` is a ROS 2 package for the Humble distribution that monitors multiple topics of a specific message type and checks for message activity. It reads a list of topics and their message type from a YAML configuration file and tracks whether messages are being published to each topic within a specified timeout. It then publishes a JSON-formatted status report.

## 🧠 Features

* Monitors multiple ROS 2 topics of a specified message type.
* Detects when no messages have been received on a topic within a user-defined timeout.
* Publishes a real-time status summary to the `/multi_topic_monitor` topic as a JSON message.
* Lightweight and configurable.

## 📦 Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/patrickstratznig/multi_topic_monitor.git
cd ~/ros2_ws
colcon build --packages-select multi_topic_monitor
source install/setup.bash
```

## ⚙️ Configuration

The package requires a YAML configuration file that defines:

* A list of topics to monitor.
* The shared message type.
* The timeout (in seconds).

Example `config/multi_topic_monitor_config.yaml`:

```yaml
timeout: 5.0
topics:
  /front_camera/rgb/image_raw: sensor_msgs/msg/Image
  /top_lidar/points: sensor_msgs/msg/PointCloud2
```

## 🚀 Usage

Launch the monitor node using:

```bash
ros2 run multi_topic_monitor multi_topic_monitor ~/ros2_ws/src/multi_topic_monitor/config/multi_topic_monitor_config.yaml
```

## 📤 Output

The node publishes a status message to `/multi_topic_monitor` as a `std_msgs/msg/String` message containing a JSON object like:

```json
{
  "/front_camera/rgb/image_raw": 1,
  "/top_lidar/points": 0
}
```

* `1` = Topic is active (messages received within timeout).
* `0` = Topic is inactive (no messages received within timeout).

## 🧩 Implementation Notes

* Message types are dynamically imported using Python’s importlib.
* Subscriptions use `QoSReliabilityPolicy.BEST_EFFORT` for compatibility with sensors like cameras and lidars.
* Status is checked and published every 4 seconds (can be changed via `create_timer` interval).

## 🛠 Dependencies

* ROS 2 Humble
* Python 3
* `rclpy`
* `std_msgs`

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙌 Contributions

Contributions, issues, and feature requests are welcome! Feel free to open a pull request or issue.
