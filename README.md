# IAAC MRAC WORKSHOP 3.2 - Drafting Deception #

## Installation ##
Download and install Unity Version [2019.3.6](https://unity3d.com/get-unity/download/archive)

This package includes [ROS#](https://github.com/siemens/ros-sharp), a set of open source software libraries and tools for [ROS](http://www.ros.org/) communication.

## WSL2 Setup ##
For testing on a local windows device without a second Ubuntu machine or prior WSL/ROS install.  Summary mostly from [here](https://jack-kawell.com/2020/06/12/ros-wsl2/).

Install [WSL2](https://docs.microsoft.com/en-us/windows/wsl/install-win10)

Run Powershell as administrator

```shell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```
```shell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

Restart Machine, and then in powershell again:


```shell
wsl --set-default-version 2
```
If prompted to update kernel, follow link and download/run .msi, running above command again after.

Get and install [Ubuntu 18.04 LTS via the Microsoft Store](https://www.microsoft.com/store/apps/9N9TNGVNDL3Q).  Launch after installation, and set username and password.



<<<TEMP JEFF HERE>>>



For easy access of Ubuntu terminal, powershell, etc. use [Windows Terminal](https://aka.ms/terminal).

Download and install [VcXsrv]https://sourceforge.net/projects/vcxsrv/files/latest/download) for gui forwarding.  Run X-Launch from the start menu, and use the default settings apart from **unchecking** *native opengl* and **checking** *disable access control*.  Save the configuration file before clicking finish to use this as a shortcut for running with these settings in the future.

Add your windows machine's ip address (replace `{your_ip_address}`) to your bashrc by running (in ubuntu console)
(If new to this, note you need to use *SHIFT+CTRL+C* for pasting in Ubuntu terminal)

```shell
echo 'export DISPLAY={your_ip_address}:0.0' >> ~/.bashrc
```
```shell
source ~/.bashrc
```
```shell
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

### ROS INSTALL ###

```shell
sudo apt update && sudo apt upgrade
```
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```shell
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
```shell
sudo apt update && sudo apt install ros-melodic-desktop-full
```
```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```shell
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init && rosdep update
```
 Test install by starting roscore in one ubuntu terminal
```shell
roscore
```
and rviz in another 
```shell
rosrun rviz rviz
```

### Playing Rosbags ###
First install rosbridge server in ubuntu
```sudo apt-get install ros-melodic-rosbridge-server```

Copy the *bags* directory from this repo in the windows browser into `\\wsl$\Ubuntu-18.04\home\{ubuntu_username}`

In separate terminals:
```roscore```
```roslaunch rosbridge_server rosbridge_websocket.launch```
```cd ~/bags && rosbag play ur_driver.bag -l```

Run the unity scene and you should see the robot move a bit.

### URSIM Install ###
Create an account and download [ursim](https://www.universal-robots.com/download/software-e-series/simulator-linux/offline-simulator-e-series-ur-sim-for-linux-582/) for the closest robot version.

Copy this file into your ubuntu home directory in the windows browser `\\wsl$\Ubuntu-18.04\home\{ubuntu_username}`

Then in ubuntu terminal run (change file and folder names if using a different usrsim version.)

Get Default JRE/JDK
```shell
sudo apt-get update && sudo apt-get install openjdk-8-jdk
```

Set JDK version to 1.8 (enter correct number)
```sudo update-alternatives --config java```

```shell
cd ~
tar xvzf URSim_Linux-5.8.2.10297.tar.gz
cd ursim-5.8.2.10297
./install.sh
```

For me this install failed in WSL 2 and I manually installed many of the dependencies and changed the directory for shortcuts in *install.sh*  Hopefully this is not the case for you...(steps not documented here).

run with 
```shell
cd..
ursim-5.8.2.10297/start-ursim.sh UR10
```

Once you are able to successfully launch ursim, you need to add the URCap in:


## Workshop ##
This unity package is developed in connection to the Drafting Deception workshop tutored by [Jeffrey Anderson](https://jeffandarch.com/) and [Ryan Luke Johns](http://www.greyshed.com/), together with [Daniil Koshelyuk](https://daniil.koshelyuk.site/) and [Soroush Garivani](http://iaac.net/mrac).

