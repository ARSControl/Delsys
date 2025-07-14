**# Trigno Sensor Recorder**

This project is an asynchronous Python script for recording data from Trigno sensors using the **AeroPy** library. 
It streams the sensor data via TCP sockets in JSON format and also saves it to CSV files for offline analysis.



**# Requirements**

- Python 3.8+
- Python libraries:
  - `keyboard`
  - `asyncio`
  - `csv` 
  - `json` 
  - `AeroPy` (with `TrignoBase` and `DataKernel` modules)



**# Usage**

Press s to start recording.

Press q to stop recording.



**# Customization**

In the main section at the bottom of the script, insert the **IP address** of the Linux host running the ROS node.

You can change the sensor **acquisition mode** by modifying the self.desired_mode variable in the TrignoRecorder class.
