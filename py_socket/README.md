# PYTHON SOCKET
## Socket package
This package handles rading and writing of data coming from socket connection. Data is read and then is published on dedicated topics.

The topics advertised can be found within the **port_topic_map** dictionary of the **ReadDataNode** class.

Data published on the topics is of type custom and information on data structure can be found within the **Delsys Interfaces** package.

Parameters to chose if setting a timer on the node operating state can be set running the launch file. 
The parameters are:
- **timer_flag** --> Flag to enable/disable timer (default: False)
- **timer_value** --> timer value in seconds (default: 60 s)
