import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import json

# Import custom message types
from delsys_interfaces.msg import DelsysSensorData, DelsysSensorInfo


# This class defines a ROS2 node that reads data from TCP sockets and publishes it to ROS2 topics.
class ReadDataNode(Node):
    def __init__(self):
        super().__init__('socket_reader_node')

        # Declare and get parameters for an optional timer to control the node's runtime.
        self.declare_parameter('timer_value', 60)
        self.declare_parameter('timer_flag', False)

        self.timer_duration = self.get_parameter('timer_value').get_parameter_value().integer_value
        self.flag_timer = self.get_parameter('timer_flag').get_parameter_value().bool_value

        self.get_logger().info(f"Timer flag: {self.flag_timer}, Timer duration: {self.timer_duration}s")

        # Map TCP ports to corresponding ROS2 topic names for data publishing.
        self.port_topic_map = {
            9001: 'info_data',
            9002: 'emg_data',
            9003: 'acc_data',
            9004: 'gyro_data',
        }

        self.port_publishers = {}
        self.servers = []

        # Create a publisher for each port-topic pair defined in the map.
        for port, topic in self.port_topic_map.items():
            if topic == 'info_data':
                self.port_publishers[port] = self.create_publisher(DelsysSensorInfo, topic, 1000)
            else:
                self.port_publishers[port] = self.create_publisher(DelsysSensorData, topic, 1000)
 
    # Starts the TCP servers and the ROS2 node spinning asynchronously.
    async def start(self):
        await self.start_servers()
        asyncio.create_task(self.spin_ros())

    # Asynchronously spins the ROS2 node to process callbacks.
    async def spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)  # set 0 or lower than asyncio.sleep()
            await asyncio.sleep(0.01)

    # Populates and publishes a DelsysSensorInfo message.
    def publish_info_data(self, port, msg):
        ros_msg = DelsysSensorInfo()
        ros_msg.sensor = msg['Sensor']
        ros_msg.channel = msg['Channel']
        ros_msg.guid = msg['GUID']
        ros_msg.data_type = msg['DataType']
        self.port_publishers[port].publish(ros_msg)
        self.get_logger().info(f"[PORT {port}] Published: {msg}")

    # Populates and publishes a DelsysSensorData message.
    def publish_data(self, port, msg):
        ros_msg = DelsysSensorData()
        ros_msg.data_type = msg['DataType']
        ros_msg.guid = msg['GUID']
        ros_msg.time_stamp = msg['TimeStamp']
        ros_msg.data_value = msg['DataValue']
        self.port_publishers[port].publish(ros_msg)
        self.get_logger().info(f"[PORT {port}] Published: {msg}")

    # Initializes and starts an asyncio TCP server for each port.
    async def start_servers(self):
        for port, topic in self.port_topic_map.items():
            server = await asyncio.start_server(
                self._make_handler(port, topic),
                host='0.0.0.0',
                port=port
            )
            self.servers.append(server)
            self.get_logger().info(f"[PORT {port}] Async TCP server started")

    # Creates a handler for processing incoming data from a TCP client.
    def _make_handler(self, port, topic):
        async def handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
            addr = writer.get_extra_info('peername')
            self.get_logger().info(f"[PORT {port}] New connection from {addr}")
            try:
                # Continuously read data from the client connection.
                while True:
                    data = await reader.readline()
                    if not data:
                        break
                    msg_str = data.decode('utf-8').strip()
                    msg = json.loads(msg_str)

                    if not isinstance(msg, dict):
                        raise Warning(f"Received data is not a dictionary. Received: {msg}")

                    # Publish data to the appropriate topic based on the port.
                    if topic == 'info_data':
                        self.publish_info_data(port, msg)
                    else:
                        self.publish_data(port, msg)

            except Exception as e:
                self.get_logger().error(f"[PORT {port}] Error with client {addr}: {e}")
            finally:
                # Cleanly close the client connection.
                writer.close()
                await writer.wait_closed()
                self.get_logger().info(f"[PORT {port}] Connection from {addr} closed")
        return handler

    # Closes all active servers before destroying the node.
    def destroy_node(self):
        for server in self.servers:
            server.close()
        return super().destroy_node()


# An asynchronous loop that keeps the main program alive while rclpy is running.
async def main_loop():
    while rclpy.ok():
        await asyncio.sleep(0.1)


# Main asynchronous function to initialize and run the ROS2 node.
async def main_async():
    rclpy.init()
    node = ReadDataNode()
    await node.start()

    try:
        # If the timer is enabled, the node runs for a specified duration.
        if node.flag_timer:
            node.get_logger().info(f"Node will run for {node.timer_duration} seconds.")
            await asyncio.wait_for(main_loop(), timeout=float(node.timer_duration))
        else:
            # Otherwise, it runs until manually interrupted (Ctrl+C).
            node.get_logger().info("Node will run until manually stopped (Ctrl+C).")
            await main_loop()
    except (asyncio.CancelledError, asyncio.TimeoutError):
        # Handles shutdown when the timer expires.
        node.get_logger().info("Timer expired, shutting down.")
    finally:
        # Ensures node resources are properly released on exit.
        node.destroy_node()
        rclpy.shutdown()


# Entry point of the script.
def main():
    try:
        # Runs the main asynchronous event loop.
        asyncio.run(main_async())
    except KeyboardInterrupt:
        # Handles manual shutdown via Ctrl+C.
        print("Shutdown requested by user.")


if __name__ == '__main__':
    main()

