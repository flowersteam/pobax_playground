# POBAX Playground
## Install package and dependencies

1. Setup [ROS and Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
```
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
# Download baxter setup script:
cd ~/catkin_ws
wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
chmod u+x baxter.sh
```

2. Clone the dependencies and compile:
```
cd ~/catkin_ws/src  # Or ros_ws if you created such workspace
git clone https://github.com/flowersteam/thr_infrastructure.git

# The packages below are deps of THR Infrastructure
git clone https://github.com/flowersteam/baxter_pykdl.git
git clone https://github.com/flowersteam/optitrack_publisher.git -b calibrations
git clone https://github.com/flowersteam/baxter_commander.git
git clone https://github.com/flowersteam/trac_ik_baxter.git
git clone https://github.com/flowersteam/trac_ik.git
sudo apt install libnlopt-dev libnlopt0 ros-kinetic-nlopt
sudo apt install python-pip ros-kinetic-moveit
sudo pip install xmltodict
# Don't forget to also add the pobax_playground repository 

cd ~/catkin_ws
catkin_make
```
3. Install dependencies specific to the pobax playground 
```
sudo pip install pypot
sudo pip install poppy-torso
sudo apt-get install octave
sudo pip install oct2py
sudo pip install pyaudio

# Building wheel fails for pyaudio ? Try this:
sudo apt install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 ffmpeg libav-tools
```
4. Download [the latest Diva](http://sites.bu.edu/guentherlab/software/diva-source-code/) and unzip files in ~/software/DIVAsimulink (or change default directory in voice.py)

`thr_infrastructure` (Third hand robot infrastructure) is the Baxter part inherited from the Third Hand Project.
Checkout [more details about `thr_infrastructure` online](https://github.com/flowersteam/thr_infrastructure#third-hand-robot-infrastructure).


## Start

1. Make sure baxter and the optitrack pc are both launched and detected (use ping cmd to check)
2. Sync your clock with baxter: ntpdate -q ntp.ubuntu.com
3. Calibrate optitrack and baxter using the calibration notebook in [optitrack_publisher](https://github.com/flowersteam/optitrack_publisher#calibrate)
4. Use [the update_scene_assistant notebook](https://github.com/flowersteam/thr_infrastructure/tree/master/thr_scenes/config/pobax) to setup baxter's grasp and place poses. You can also directly edit thr_infrastructure/thr_scenes/config/pobax/poses.json.
5. You can now test manually if baxter is able to grasp and replace the culbuto
```
cd ~/catkin_ws
./baxter.sh   # make sure you edited its variables first (baxter_hostname, your_hostname and ros_version)
roslaunch thr_interaction_controller manual.launch scene:=pobax display:=action
# A promt should appear in the terminal, from which you can send the following commands to baxter:
r (reset right arm to starting position)
g1 (grasp culbuto)
p1 (place culbuto)
```

6. If everything worked so far you should be able to launch the pobax_playground experiment:
```
roslaunch pobax_playground start.launch name:=demo iterations:=20000
```
## Troubleshooting
### Object grasping misses precision
Please recalibrate Optitrack using the wanding bar in Motive *and* [the Baxter-Optitrack calibration](https://github.com/flowersteam/optitrack_publisher#calibrate).

### Could not load the xml from parameter server: /robot_description
have you executed `./baxter.sh`? it looks like you're using a local ROS Master instead of Baxter's one.

### Cannot open dev/ttyacm0: permission denied
Quick solution: sudo chmod 666 /dev/ttyACM0
see [this stackoverflow post](https://stackoverflow.com/questions/27858041/oserror-errno-13-permission-denied-dev-ttyacm0-using-pyserial-from-pyth) for more details and a permanent solution.
