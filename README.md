# WORSPACE IN PROGRESS - DELSYS API SOCKET
This repo is meant to be used with the official Delsys API [ADD URL] and ros2 (developing on Humble distro).

CONTENT:
- **windows_socket**: repo containing all necessary files to be used with the Delsys API and send data over socket connection (Windows side)
- **py_socket**: ros2 package to read data over socket connection (Linux side)
- **py_data_handler**: ros2 package to handle data read by py_socket


# TO DO
- write README.md for each repo

## Linux 
- set timer to start when datastream starts --> now the timer starts as soon as the node starts

## Windows
- organize the data sent over socket in a nested-dictionary fashion --> for each datatype (emg, acc or gyro), the data should be divided into 4 parts: 
    - header: string
    - id: string
    - time stamp: float
    - data value: float
- --> each datatype will then belong to a certain sensor, which will be part of the main dictionary that will contain the key-value pairs of the used sensors
- --> the final structure should look something like:
 ```json
   {
       "sensor1": {
           "datatype1": {
               "header": "string",
               "id": "string",
               "time_stamp": "float",
               "data_value": "float"
           },
           "datatype2": {
               "...": "..."
           },
           "...": "..."
       },
       "sensor2": {
           "...": "..."
       },
       "...": "..."
   }
   ```
- remove shutdown timer
- add routine to shutdown socket if connection is interrupted and/or closed by Linux side

