# esp_8266_rosserial
connect Ros to Esp 8266 module for master slave communication over wifi.

# Arduino IDE Setup 
### Install Arduino IDE
- For linux install via snap insted of Apt
```sh
sudo snap install arduino
```
### Install Esp 8266 Board
- Go to preferences from file and add additional boards.
[![N|Solid](https://img-blog.csdnimg.cn/20200930084133286.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L1poYW5nUmVsYXk=,size_16,color_FFFFFF,t_70)]()
- add these two line there
```
https://dl.espressif.com/dl/package_esp32_index.json,
http://arduino.esp8266.com/stable/package_esp8266com_index.json
```
- then install Following boards required
- [![N|Solid](https://img-blog.csdnimg.cn/20200930084448779.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L1poYW5nUmVsYXk=,size_16,color_FFFFFF,t_70)]()

### Set up Ros libraries needed
- Install Ros Package for Rosserial
```sh
sudo apt-get remove ros-[Distro]-rosserial*
```
- Goto   ~/sketchbook/libraries location and run following command
```sh 
rosrun rosserial_arduino make_libraries.py .
```
# Add Code to Esp
- Use Esp_code/BuiltInLed_Ros.ino file to upload to esp.
- Change Wifi Name and pass And set Ip on which your ros host pc is running.

# Setup Ros On pc 
- Create Ros Publisher Node use Ros_pub/python_ros_chat.py file
- Run Roscore First
- following command to connect to Esp via Wifi
```sh
rosrun rosserial_python serial_node.py tcp
```
- now launch Python file to Pub msgs to Topic


## License

MIT