# POBAX Playground
## Install package and dependencies

1. Setup [ROS and Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) (we're assuming you're on ROS Kinetic)
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

# Setup baxter workstation (see http://sdk.rethinkrobotics.com/wiki/Workstation_Setup for details):
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
# Download baxter setup script:
cd ~/catkin_ws
wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
chmod u+x baxter.sh

cd ~/catkin_ws
catkin_make
```

## Start
```
cd ~/catkin_ws
./baxter.sh   # edited its variables first (baxter_hostname, your_hostname and ros_version)
roslaunch thr_interaction_controller manual.launch scene:=pobax display:=action
```

`thr_infrastructure` (Third hand robot infrastructure) is the Baxter part inherited from the Third Hand Project.
Checkout [more details about `thr_infrastructure` online](https://github.com/flowersteam/thr_infrastructure#third-hand-robot-infrastructure).

## Troubleshooting
### Object grasping misses precision
Please recalibrate Optitrack using the wanding bar in Motive *and* [the Baxter-Optitrack calibration](https://github.com/baxter-flowers/optitrack_publisher#calibrate).

### Could not load the xml from parameter server: /robot_description
have you executed `./baxter.sh`? it looks like you're using a local ROS Master instead of Baxter's one.
