# Import necessary modules
import asyncio  # For asynchronous operations
import time     # For time tracking
import csv      # For writing data to CSV
from collections import defaultdict  # For dictionary with default list
import keyboard  # For keyboard input detection

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
        # Establish TCP connection to the given host and port
        reader, writer = await asyncio.open_connection(self.host, self.port)
        self.writer = writer

    async def send(self, message):
        try:
            # Send encoded message through socket
            self.writer.write(message.encode())
            await self.writer.drain()
        except Exception as e:
            print(f"Socket send error on port {self.port}: {e}")

    # FUNCTION NOT CURRENTLY USED --> KEEP IT TO ALLOW USER TO STOP THE SOCKET ON WINDOWS SIDE OR LEAVE CONTROL TO UBUNTU??
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

        self.global_start_time = None
        self.recorded_data = [] # Stores data for CSV

         # Initialize socket connections for each data type
        self.sockets = {dtype: SensorSocket(self.HOST, port) for dtype, port in self.PORTS.items()}

        self.base = None # TrignoBase instance (will be initialized)
        self.guids = defaultdict(list) # Store GUIDs for each sensor type
        self.queues = {dtype: asyncio.Queue() for dtype in ['EMG', 'ACC', 'GYRO']} # Queues for async processing

        self.sensor_info_lines = []  # Store sensor info for CSV header

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
                info_line = f"Sensor {sensor.Id}, Channel {channel.Name}, GUID {channel.Id}, Type {channel.Type}"
                self.sensor_info_lines.append(info_line) # Store sensor metadata

                # Store channel GUIDs for each data type
                if ch_type in self.guids:
                    self.guids[ch_type].append(channel.Id)
                if ch_type in self.queues:
                    self.guids[ch_type].append(channel.Id)

                # Optionally send sensor metadata via INFO socket
                self.sockets['INFO'].writer.write((info_line + "\n").encode())

        return sensors

    # Process a queue for a specific sensor type (EMG, ACC, GYRO)
    async def process_queue(self, dtype):
        sock = self.sockets[dtype]
        queue = self.queues[dtype]
        while True:
            guid, data = await queue.get() # Get next data item from queue
            for sample in data:
                # Format data sample as CSV line
                message = f"{dtype},{guid},{sample.Item1},{sample.Item2}\n"
                await sock.send(message) # Send to socket

                # Also store in memory for later CSV export
                self.recorded_data.append({
                    "sensor_type": dtype,
                    "channel_guid": guid,
                    "time_point": sample.Item1,
                    "value": sample.Item2
                })

    # Start recording sensor data for a given duration 
    # duration could be change
    async def record(self, duration=60):
        self.global_start_time = time.time()
        self.base.TrigBase.Start(ytdata=True)
        print("Recording started. Press 'q' to stop.")

        while True:

            # --------------- ADD ON ROS SIDE: choices for flag limited stream time and amount ---------------
            # elapsed = time.time() - self.global_start_time
            # # Stop if time is up or user presses 'q'
            # if elapsed > duration or await asyncio.to_thread(keyboard.is_pressed, 'q'):
            #     print("Stopping recording.")
            #     break

            # If new data is available
            if self.base.TrigBase.CheckYTDataQueue():
                yt_data = self.base.TrigBase.PollYTData()
                for guid, data in yt_data.items():
                    # Route data to appropriate queue
                    for dtype, guids in self.guids.items():
                        if guid in guids:
                            await self.queues[dtype].put((guid, data))

            await asyncio.sleep(0.001) # Prevent busy-waiting

        # Stop data collection and reset system
        # non ci entra ma chiudere la connessione se la socket da linux si chiude 
        self.base.TrigBase.Stop()
        self.base.TrigBase.ResetPipeline()
        print("Recording complete.")

    # Write recorded data and sensor info to CSV
    def save_to_csv(self, file, sensors):
        with open(file, "w", newline="") as f:
            # Write sensor metadata block
            for line in self.sensor_info_lines:
                f.write(line + "\n")
            f.write("\n")  # Empty line before data block
            
            # Write data block
            writer = csv.DictWriter(f, fieldnames=["sensor_type", "channel_guid", "time_point", "value"])
            writer.writeheader()
            writer.writerows(self.recorded_data)

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

        file = "recorded_data.csv"
        self.save_to_csv(file, sensors) # Save data to file
        print(f"Successfully saved stream data to {file}")

# -----------------------------------
# Entry point for running the script
# -----------------------------------
if __name__ == "__main__":
    # Create a TrignoRecorder instance and run its async pipeline
    # IP change according to IP remote host
    asyncio.run(TrignoRecorder("192.168.0.156").run())