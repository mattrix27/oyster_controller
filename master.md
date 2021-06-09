# Oystamaran/Flippy Overview

Hello! If you're reading this, you're probably interested in working on/with this project run by MIT Seagrant in cooperation with Ward Aquafarms and the MIT course 2.017 (Design of Electromechanical Robotic Systems) 2020 and 2021. In an effort to automate the flipping process for the oyster bags, I have added this to outline the work done so far and give details on the proposed plans for this process. A fair bit of this has not been tested and will definitely needed to be further optimizaed and tested. If there are any possible improvements that you can think of, please feel free!

## For more info:
<ul>
  <li><a href="https://drive.google.com/file/d/1_88Q87PlqoZSkVqVxlvEns5nIxa_vscF/view?usp=sharing">My thesis</a></li>
  <li><a href="">2.017 Final Paper 2021</a> (provides more background and details on the mechanical and electrical side of the ASV</li>
  <li><a href="">2.017 Final Paper 2020</a> (What was proposed in the beginning fabrication of the project)</li>
</ul>


## Usage

The main infrastructure of the code were developed with ROS and MOOS-IvP, so a background in both would definitely be recommended to work on it (as well as their languages, Python and C++, respectively). The main software components are run as nodes that should wrap them all together. It is currently held on a Jetson TX2 with Jetpack 4.5.1 installed, which provides an Ubuntu 18.04 OS. 

1. Connect the Front Facing ZED camera. we use a model ZED1 camera, a popular depth camera that comes with ROS libraries. It connects via USB and should just require a simple connection.
<ul>
  <li>If you wish to run solo via ROS, you can run:
    
    roslaunch zed_wrapper zed.launch
    
  </li>
  <li>If you just want to record video for logging data, you can run:
    
    python3 /home/oyster/zed-examples/svo\ recording/recording/python/svo_recording.py
    
  </li>
  <li>For more info or details on how to work with ROS, or in general, you can look at the <a href="https://www.stereolabs.com/docs/">ZED Documentation</a>
  </li>
</ul>

2. Connect to Bottom Facing Go Pro camera. We use a GoPro Hero 4 currently and stream the data via ROS with https://github.com/mattrix27/ros-gopro-driver.
<ul>
  <li> A secondary USB Wifi Adapter is required and should auto-connect to the GoPro's Wifi Network <strong>NOTE: THIS HAS BEEN BUGGY RECENTLY, SHOULD BE FURTHER TESTED AND/OR WIFI DONGLE NEEDS TO BE REPLACED BEFORE IN FIELD USE</strong></li>
</ul>

3. Attach RC. Currently, we use an XBox 360 Controller that connects via USB Dongle. It uses the [SDL2 library](https://www.libsdl.org/) to communicate via MOOS-IvP to send commands. 

### Connecting 

Once you have all of your sensors connected and the Jetson TX2 is powered up, you can start working with and operating the device. 

**Username:** `oyster`

**Password:** `oyster`

When in lab, you can connect just by connecting a HDMI display and mouse and keyboard. This would allow you see video streams and work smoothly when in the lab setting. 

#### SSH via USB:

The TX2 has a micro-USB port that you can plug into using a USB-to-micro-USB cable to your computer. This should automatically cause the L4T protocol to startup and create an IP address. The default should be: `192.168.55.1`. You can also run `ifconfig` to see if any new entries have popped up and doublecheck. With that, once connected you should be able to ssh in with the command: `ssh oyster@192.168.55.1`, or whatever your IP is.

#### SSH via AP:

With the Jetson TX2 you are able to make the TX2 use it's built wireless antennae as a Wireless Access Point. This has been implemented already but for re-creating purposes.

1. Edit the file `/etc/modprobe.d/bcmdhd.conf` and add the line: `options bcmdhd op_mode=2`
2. Reboot the computer
3. Go to Wifi Settings and Add a Hotspot connection (name it whatever you want, you can also set a manual IP address to remember it)
4. You should know be able to find an IP address, (or the manual one you set), use `ifconfig` to doublecheck, connect to the created hotspot Wifi network and SSH like you would with USB. 
5. If it still doesn't work, you can try going to Wifi settings and click the `Turn On WiFi Hotspot` and try again

Now, you have the TX2 as a WAP. In this setting, you normally cannot access the internet normally, so if you finish to toggle between you can just comment and uncomment the `options bcmdhd op_mose=2` line. Currently, I set it up so that the WiFi network is called `oyster-desktop`, so you can connect to that network on your computer (the password is **oyster21**) and the IP is set to `192.168.121.10` so you can ssh with the command: `ssh oyster@192.168.131.10`.

### Quickstart:

On this current computer, I set aliases in the `~/.bashrc` file to make it easier to run the set commands. These can be changed, modified, or the full commands can always be run.

If you are working directly on the computer or you plan to always stay in range or the computer. You can just open terminals, ssh in, and run these commands. 
If not, you can use the **scren** library to create sessions and detach them so that commands continue to run even when the connection is lost.

1.`screen` to start a session
2. Run the desired command
3. Detach the session by running `Ctrl-A + Ctrl-D`
4. When you reconnect you can reattach session by running `screen -r [screen session ID (if multiple sessions are running)]`

To begin basic RC functionality, each command should currently be run in its own terminal and you should wait for it to finish loading.

1. Make the controller and everything are connected
2. `pAntler_oyster` or `pAntler /home/oyster/moos-ivp-oyster/missions/oyster/master.moos`, this starts the MOOS Apps that control the RC and the Driving, as well as the NMEA client to communicate with ROS.
3. Press the button 3 (currently **X** on the controller), to have the controller be found.
4. `ros_moos_bridge` or `roslaunch ros_moos_bridge ros_moos_bridge.launch`, this starts the ROS node to connect to created NMEA client (see, order matters).
5. `ros_arduino1` or `ros_arduino0`, this will run `rosrun rosserial_arduino serial_node.py /dev/ttyACM[X]` where the `[X]` is the port number in the command, this connects the ROS to the flipper Arduino
6. `oyster_controller` or `roslaunch oyster_controller oyster_controller.launch` This starts the FSM and the connection to the Flipper arduino.

From here you should be able to control the boat via RC, just hold down the right bumper on the controller when giving commands, use the left joystick to move the motors, **A** to flip, **Y** to go to the IDLE state, and **B** to go the FRONT state.

If we wish to go start the autonomy component:

To run the perception stack, you can run: `bag_detection` or `roslaunch bag_detection bag_detection.launch` to start and run both cameras and begin processing.

If you wish to focus/test on one part/camera, you can run the sub-launch files.

`roslaunch bag_detection bag_flip_detection.launch` to start the GoPro ROS node and the bottom-facing perception

`roslaunch bag_detection bag_path_detection.launch` to start the ZED camera and the front-facing perception.
