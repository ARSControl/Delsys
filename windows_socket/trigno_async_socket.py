# Import necessary modules
import asyncio  # For asynchronous operations
import csv      # For writing data to CSV
from collections import defaultdict  # For dictionary with default list
import keyboard  # For keyboard input detection
import os
import json

# Import Trigno-related interfaces from AeroPy library
from AeroPy.TrignoBase import TrignoBase
from AeroPy.DataManager import DataKernel

# -----------------------------------
# Class to handle socket communication
# -----------------------------------
class SensorSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.writer = None

    async def connect(self):
        try:
            # Establish TCP connection to the given host and port
            reader, writer = await asyncio.open_connection(self.host, self.port)
            self.writer = writer
            print(f"Port {self.port} connected to host {self.host}")
            return True
        except (ConnectionError, ConnectionResetError, BrokenPipeError) as e:
            print(f"Error connecting to port {self.port}: {e}")
            self.close()
            print(f"Connection on port {self.port} closed.")
            return False
        
        
    async def send(self, message):
        try:
            # Send encoded message through socket
            data = json.dumps(message) + "\n"
            self.writer.write(data.encode('utf-8'))            
            await self.writer.drain()
        except (ConnectionError, ConnectionResetError, BrokenPipeError) as e:
            print(f"Socket send error on port {self.port}: {e}")
            self.close()
            print(f"Connection on port {self.port} closed.")
            return False

    def close(self):
       # Close the socket connection
       if self.writer:
           self.writer.close()

# -----------------------------------
# Main class for managing Trigno sensors
# -----------------------------------
class TrignoRecorder:
    def __init__(self, host):
        self.HOST = host
        # Ports for different sensor data types
        self.PORTS = { 'INFO': 9001, 'EMG': 9002, 'ACC': 9003, 'GYRO': 9004}
        
        # Desired configuration for sensors
        # configuration could be change according to new mode selected from csv filtered file
        self.desired_mode = 'EMG raw (4000 Hz), skin check (74 Hz), ACC 2g (74 Hz), GYRO 250 dps (74 Hz), +/-11mv, 10-850Hz'

         # Initialize socket connections for each data type
        self.sockets = {dtype: SensorSocket(self.HOST, port) for dtype, port in self.PORTS.items()}

        self.base = None # TrignoBase instance (will be initialized)
        self.guids = defaultdict(list) # Store GUIDs for each sensor type
        self.queues = {dtype: asyncio.Queue() for dtype in ['EMG', 'ACC', 'GYRO']} # Queues for async processing

        # Store data to write to csv
        self.recorded_data_dicts = [] 
        self.sensor_info_dicts = []  

    # Set up Trigno base interface
    def setup_base(self):
        self.base = TrignoBase(None)
        self.base.collection_data_handler = DataKernel(self.base)

    # Connect to all sensor data sockets
    async def connect_sockets(self):
        await asyncio.gather(*(sock.connect() for sock in self.sockets.values()))

    # Discover sensors and apply configurations
    def configure_sensors(self):
        sensors = self.base.Scan_Callback() # Detect connected sensors
        for i, sensor in enumerate(sensors):
            self.base.TrigBase.SetSampleMode(i, self.desired_mode) # Apply configuration

        for sensor in sensors:
            for channel in sensor.TrignoChannels:
                ch_type = str(channel.Type)
                info_dict = {"Sensor": str(sensor.Id), "Channel": str(channel.Name), "GUID": str(channel.Id), "DataType": ch_type}
                self.sensor_info_dicts.append(info_dict) # Store sensor metadata

                # Store channel GUIDs for each data type
                if ch_type in self.guids:
                    self.guids[ch_type].append(channel.Id)
                if ch_type in self.queues:
                    self.guids[ch_type].append(channel.Id)

                # Send sensor metadata via INFO socket --> this line does exactly the same thing async def send()
                # CHANGE IT? LEAVE IT LIKE THAT?
                self.sockets['INFO'].writer.write((json.dumps(info_dict) + "\n").encode('utf-8'))
        return sensors

    # Process a queue for a specific sensor type (EMG, ACC, GYRO)
    async def process_queue(self, dtype):
        sock = self.sockets[dtype]
        queue = self.queues[dtype]
        while True:
            guid, data = await queue.get() # Get next data item from queue --> .get() method flushes the elements present in the asyncio.Queue() object from the left most and empty the queue
                                            # --> if the Queue() object is empty, it waits until new data is queued
            for sample in data:
                # Format data sample as CSV line
                message = {"DataType": dtype, "GUID": str(guid), "TimeStamp": sample.Item1,"DataValue": sample.Item2}    
                await sock.send(message) # Send to socket

                # Also store in memory for later CSV export
                self.recorded_data_dicts.append({
                    "sensor_type": dtype,
                    "channel_guid": guid,
                    "time_stamp": sample.Item1,
                    "value": sample.Item2
                })

    # Start recording sensor data for a given duration 
    async def record(self):
        self.base.TrigBase.Start(ytdata=True)
        print("Recording started. Press 'q' to stop.")

        while True:

            # Stop if time is up or user presses 'q'
            if await asyncio.to_thread(keyboard.is_pressed, 'q'):
                print("Stopping recording.")
                break

            # If new data is available
            if self.base.TrigBase.CheckYTDataQueue():
                yt_data = self.base.TrigBase.PollYTData()
                for guid, data in yt_data.items():
                    # Route data to appropriate queue
                    for dtype, guids in self.guids.items():
                        if guid in guids:
                            await self.queues[dtype].put((guid, data))  # .put() method append to the right end of the queue - i.e. asyncio.Queue() object

            await asyncio.sleep(0.001) # Prevent busy-waiting

        # Stop data collection and reset system
        self.base.TrigBase.Stop()
        self.base.TrigBase.ResetPipeline()
        print("Recording complete.")

    # Write recorded data and sensor info to CSV
    def save_to_csv(self, file_path, data_dicts_list):
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, "w", newline="") as f:
            # Write block
            writer = csv.DictWriter(f, fieldnames=[key for key in data_dicts_list[0].keys()])
            if not data_dicts_list:
                print(f"[WARNING] No data to save to {file_path}")
            writer.writeheader()
            writer.writerows(data_dicts_list)            

    # Full process: setup → wait → record → save
    async def run(self):
        self.setup_base() # Initialize base connection
        await self.connect_sockets() # Connect all data sockets
        self.base.Connect_Callback() # Connect to sensors
        sensors = self.configure_sensors() # Configure sensors
        self.base.TrigBase.Configure(start_trigger=False, stop_trigger=False) # Configure base

        print("Press 's' to start recording...")
        await asyncio.to_thread(keyboard.wait, 's') # Wait for user to press 's'

        # Start background tasks for processing each sensor data type
        workers = [asyncio.create_task(self.process_queue(dtype)) for dtype in self.queues.keys()]
        await self.record()  # Run main recording loop
        
        # Cancel background tasks after recording ends
        for w in workers:
            w.cancel()
            try:
                await w
            except asyncio.CancelledError:
                pass

        info_file = "info_data.csv"
        data_file = "recorded_data.csv"
        base_path = os.getcwd()
        data_folder = "csv"
        self.save_to_csv(os.path.join(base_path, data_folder, info_file), self.sensor_info_dicts) # Save info data
        print(f"Successfully saved info data to {os.path.join(base_path, data_folder, info_file)}")
        self.save_to_csv(os.path.join(base_path, data_folder, data_file), self.recorded_data_dicts) # Save recorded data
        print(f"Successfully saved stream data to {os.path.join(base_path, data_folder, data_file)}")

# -----------------------------------
# Entry point for running the script
# -----------------------------------
if __name__ == "__main__":
    # Create a TrignoRecorder instance and run its async pipeline
    # IP change according to IP remote host
    asyncio.run(TrignoRecorder("192.168.0.156").run())