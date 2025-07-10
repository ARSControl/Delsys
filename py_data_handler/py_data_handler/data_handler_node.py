import rclpy
from rclpy.node import Node
import os
import csv

from delsys_interfaces.msg import DelsysSensorData, DelsysSensorInfo

class DataHandlerNode(Node):
    def __init__(self):
        super().__init__('data_handler_node')

        # Flag params
        # self.declare_parameter('info_flag', False)
        self.declare_parameter('emg_flag', False)
        self.declare_parameter('acc_flag', False)
        self.declare_parameter('gyro_flag', False)

        # Save directory params
        self.declare_parameter('info_dir', os.getcwd())
        self.declare_parameter('emg_dir', os.getcwd())
        self.declare_parameter('acc_dir', os.getcwd())
        self.declare_parameter('gyro_dir', os.getcwd())

        # Get parameters
        # self.info_flag = self.get_parameter('info_flag').get_parameter_value().bool_value
        self.emg_flag = self.get_parameter('emg_flag').get_parameter_value().bool_value
        self.acc_flag = self.get_parameter('acc_flag').get_parameter_value().bool_value
        self.gyro_flag = self.get_parameter('gyro_flag').get_parameter_value().bool_value
        self.info_dir = self.get_parameter('info_dir').get_parameter_value().string_value
        self.emg_dir = self.get_parameter('emg_dir').get_parameter_value().string_value
        self.acc_dir = self.get_parameter('acc_dir').get_parameter_value().string_value
        self.gyro_dir = self.get_parameter('gyro_dir').get_parameter_value().string_value

        # Check if any flags are set to True
        # if not any([self.info_flag, self.emg_flag, self.acc_flag, self.gyro_flag]):
        if not any([self.emg_flag, self.acc_flag, self.gyro_flag]):
            self.get_logger().warn('No data flags are set to True. No data will be saved.')
        else:
            self.get_logger().info('Data flags are set. Data will be saved.')
        
        # Setup dirs to publish data to
        self.dirs_setup()
        
        # Subscribers
        self.subs_info = self.create_subscription(DelsysSensorInfo, 'info_data', self.subs_info_callback, 10)
        self.subs_emg = self.create_subscription(DelsysSensorData, 'emg_data', self.subs_emg_callback, 1000)
        self.subs_acc = self.create_subscription(DelsysSensorData, 'acc_data', self.subs_acc_callback, 1000)
        self.subs_gyro = self.create_subscription(DelsysSensorData, 'gyro_data', self.subs_gyro_callback, 1000)

        self.subs_info  # prevent unused variable warning
        self.subs_emg  
        self.subs_acc  
        self.subs_gyro 

        # GUIDs
        self.guids = {}

        # Log flags
        self.info_data_flag = False
        self.emg_data_flag = False
        self.acc_data_flag = False
        self.gyro_data_flag = False
        self.first_info = True

    # CALLBACKS
    def subs_info_callback(self, msg):
        # if not self.info_data_flag and self.info_flag:
        #     print("Info data is going to be saved.")
        #     self.info_data_flag = True

        # Assign GUIDs to de-multiplex data
        if "ACC" in msg.channel or "GYRO" in msg.channel:
            self.guids[msg.channel] = msg.guid

        # # Save info data to txt file
        # # if self.info_flag:
        # info_dir = self.get_parameter('info_dir').get_parameter_value().string_value
        # if not os.path.exists(info_dir):
        #     os.makedirs(info_dir)
        #     self.get_logger().info(f'Created directory: {os.path.abspath(info_dir)}')        
        file_path = os.path.join(self.info_dir, 'info_data.txt')
        if self.first_info:
            mode_str = 'w'
            self.first_info = False
        else:
            mode_str = 'a'

        try:
            with open(file_path, mode_str) as f:
                f.write(f"Sensor: {msg.sensor}\n")
                f.write(f"Channel: {msg.channel}\n")
                f.write(f"GUID: {msg.guid}\n")
                f.write(f"DataType: {msg.data_type}\n")
            self.get_logger().info(f'Saved {msg.channel} info to {os.path.abspath(file_path)}')
        except IOError as e:
            self.get_logger().error(f'Could not write to file {file_path}: {e}')

    def subs_emg_callback(self, msg):
        if not self.emg_data_flag and self.emg_flag:
            print("EMG data is going to be saved.")
            self.emg_data_flag = True
        
        if self.emg_flag:
            timestamp, value = self.unpack_emg_data_for_csv(msg)  
            file_path = os.path.join(self.emg_dir, 'emg_data.csv')
            
            try:
                with open(file_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, value])  
            except IOError as e:
                self.get_logger().error(f'Could not write to file {file_path}: {e}')

    def subs_acc_callback(self, msg):
        if not self.acc_data_flag and self.acc_flag:
            print("ACC data is going to be saved.")
            self.acc_data_flag = True
        
        if self.acc_flag:
            guid, timestamp, value = self.unpack_accgyro_data_for_csv(msg)  
            channel = next((key for key, val in self.guids.items() if val == guid), None)
            if channel is None:
                self.get_logger().warn(f"GUID {guid} not found in self.guids dictionary. Skipping message.")
                return
            file_path = os.path.join(self.acc_dir, f'{channel}.csv')
            
            try:
                with open(file_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, value])  
            except IOError as e:
                self.get_logger().error(f'Could not write to file {file_path}: {e}')

    def subs_gyro_callback(self, msg):
        if not self.gyro_data_flag and self.gyro_flag:
            print("GYRO data is going to be saved.")
            self.acc_data_flag = True
        
        if self.gyro_flag:
            guid, timestamp, value = self.unpack_accgyro_data_for_csv(msg)  
            channel = next((key for key, val in self.guids.items() if val == guid), None)
            if channel is None:
                self.get_logger().warn(f"GUID {guid} not found in self.guids dictionary. Skipping message.")
                return
            file_path = os.path.join(self.acc_dir, f'{channel}.csv')
            
            try:
                with open(file_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, value])  
            except IOError as e:
                self.get_logger().error(f'Could not write to file {file_path}: {e}')

    # def subs_gyro_callback(self, msg):
    #     if not self.gyro_data_flag and self.gyro_flag:
    #         print("GYRO data is going to be saved.")
    #         self.gyro_data_flag = True
        
    #     if self.gyro_flag:
    #         timestamp, value = self.unpack_data_for_csv(msg)  
    #         file_path = os.path.join(self.gyro_dir, 'gyro_data.csv')
            
    #         try:
    #             with open(file_path, 'a', newline='') as f:
    #                 writer = csv.writer(f)
    #                 writer.writerow([timestamp, value])  
    #         except IOError as e:
    #             self.get_logger().error(f'Could not write to file {file_path}: {e}')

    # Directories setup
    def dirs_setup(self):

        # Save info data to txt file
        # if self.info_flag:
        info_dir = self.get_parameter('info_dir').get_parameter_value().string_value
        if not os.path.exists(info_dir):
            os.makedirs(info_dir)
            self.get_logger().info(f'Created directory: {os.path.abspath(info_dir)}')        

        data_types = [
            # {'name': 'info', 'flag': self.info_flag, 'dir': self.info_dir},
            {'name': 'emg', 'flag': self.emg_flag, 'dir': self.emg_dir, 'header': ['EMG']},
            {'name': 'acc', 'flag': self.acc_flag, 'dir': self.acc_dir, 'header': ['ACC']},
            {'name': 'gyro', 'flag': self.gyro_flag, 'dir': self.gyro_dir, 'header': ['GYRO']}
        ]

        for item in data_types:
            if item['flag']:
                dir_path = item['dir']
                if not os.path.exists(dir_path):
                    os.makedirs(dir_path)
                    self.get_logger().info(f'Created directory: {os.path.abspath(dir_path)}')

                # if 'header' in item:
                if item['name'] == 'emg':
                    file_path = os.path.join(dir_path, f"{item['name']}_data.csv")
                    if not os.path.exists(file_path):
                        try:
                            with open(file_path, 'w', newline='') as f:
                                writer = csv.writer(f)
                                writer.writerow(['time_stamp'] + item['header'])
                                self.get_logger().info(f"Created file with header: {file_path}") 
                        except IOError as e:
                            self.get_logger().error(f"Could not create file {file_path}: {e}")
                else:
                    header_base = item['header'][0]
                    for ax in ['X', 'Y', 'Z']:
                        file_path = os.path.join(dir_path, f"{item['header'][0]} {ax}.csv")
                        if not os.path.exists(file_path):
                            try:
                                with open(file_path, 'w', newline='') as f:
                                    writer = csv.writer(f)
                                    writer.writerow(['time_stamp'] + [f'{header_base} {ax}'])
                                    self.get_logger().info(f"Created file with header: {file_path}")
                            except IOError as e:
                                self.get_logger().error(f"Could not create file {file_path}: {e}")

    def unpack_emg_data_for_csv(self, msg):
        time_stamp = msg.time_stamp
        data_value = msg.data_value
        return time_stamp, data_value
    
    def unpack_accgyro_data_for_csv(self, msg):
        guid = msg.guid
        time_stamp = msg.time_stamp
        data_value = msg.data_value
        return guid, time_stamp, data_value

def main():
    rclpy.init()
    node = DataHandlerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

