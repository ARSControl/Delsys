import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio


class ReadDataNode(Node):
    def __init__(self):
        super().__init__('socket_reader_node')

        self.declare_parameter('timer_value', 60)
        self.declare_parameter('timer_flag', False)

        self.timer_duration = self.get_parameter('timer_value').get_parameter_value().integer_value
        self.flag_timer = self.get_parameter('timer_flag').get_parameter_value().bool_value

        self.get_logger().info(f"Timer flag: {self.flag_timer}, Timer duration: {self.timer_duration}s")

        self.port_topic_map = {
            9001: 'info_data',
            9002: 'emg_data',
            9003: 'acc_data',
            9004: 'gyro_data',
        }

        self.port_publishers = {}
        self.servers = []

        for port, topic in self.port_topic_map.items():
            self.port_publishers[port] = self.create_publisher(String, topic, 10)
 
    async def start(self):
        await self.start_servers()
        asyncio.create_task(self.spin_ros())

    async def spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            await asyncio.sleep(0.01)

    def publish_data(self, port, msg):
        ros_msg = String()
        ros_msg.data = msg
        self.port_publishers[port].publish(ros_msg)
        self.get_logger().info(f"[PORT {port}] Published: {msg}")

    async def start_servers(self):
        for port in self.port_topic_map:
            server = await asyncio.start_server(
                self._make_handler(port),
                host='0.0.0.0',
                port=port
            )
            self.servers.append(server)
            self.get_logger().info(f"[PORT {port}] Async TCP server started")

    def _make_handler(self, port):
        async def handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
            addr = writer.get_extra_info('peername')
            self.get_logger().info(f"[PORT {port}] New connection from {addr}")
            try:
                while True:
                    data = await reader.read(1024)
                    if not data:
                        break
                    msg = data.decode(errors='replace').strip()
                    if msg:
                        self.publish_data(port, msg)
            except Exception as e:
                self.get_logger().error(f"[PORT {port}] Error with client {addr}: {e}")
            finally:
                writer.close()
                await writer.wait_closed()
                self.get_logger().info(f"[PORT {port}] Connection from {addr} closed")
        return handler

    def destroy_node(self):
        for server in self.servers:
            server.close()
        return super().destroy_node()


async def main_loop():
    while rclpy.ok():
        await asyncio.sleep(0.1)


async def main_async():
    rclpy.init()
    node = ReadDataNode()
    await node.start()

    try:
        if node.flag_timer:
            node.get_logger().info(f"Node will run for {node.timer_duration} seconds.")
            await asyncio.wait_for(main_loop(), timeout=float(node.timer_duration))
        else:
            node.get_logger().info("Node will run until manually stopped (Ctrl+C).")
            await main_loop()
    except (asyncio.CancelledError, asyncio.TimeoutError):
        # asyncio.TimeoutError is raised when the timer expires
        node.get_logger().info("Timer expired, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("Shutdown requested by user.")


if __name__ == '__main__':
    main()

