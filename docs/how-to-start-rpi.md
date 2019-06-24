# How to start with the raspberry pi

## Installing the raspebrry pi

**What you need (hardware)**
+ A PC with linux
+ A raspeberry pi
+ A micro sd
+ A micro sd adapter
+ A wifi router

**What you need (software)**

For this project we are using the image of [**Ubiquity robotics**](https://ubiquityrobotics.com/) 

>You can download the image and find the instruction for flashing the micro sd in this page [instructions](https://downloads.ubiquityrobotics.com/pi.html)

As mentioned in the instructions the image comes up as a Wifi access point, the SSID is ubiquityrobotXXXX and the password is robotseverywhere you need to connect to this wifi. 

**Once connected**

You should log in ssh with the follow command
```sh
ssh ubuntu@10.42.0.1
```
The password is `ubuntu` you maybe want to change this later

**Once logged**

You should add the wifi router ssid and password to the pi ussing the follow command
```sh
sudo pifi add "SSID" "password"
```
To avoid conflict you also should change the hostname of the rpi
```sh
sudo pifi set-hostname randomhostname
```
And reboot the rpi after that
```sh
sudo reboot
```

**Adding and testing tencendur**

Connect to you wifi router and log into the rpi with the new hostname

```sh
ssh ubuntu@randomhostname.local
```

Clone the tencendur repo and launch the install

```sh
git clone https://github.com/pedrogasg/tencendur.git

cd tencendur/

./scripts/ros_install.sh

roslaunch raspicam_node camerav2_410x308_60fps.launch
 
```