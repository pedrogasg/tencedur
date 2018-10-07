# Tencedur
Px4 Offboard interaction with mavros in raspberry py


## Update kinetic in raspberry before prociding 

```apt list --installed ros-kinetic* | grep upgradable | sed s#/.*## | xargs sudo apt install -y```

## Testing the ros node with the simulation
Go to the simulation folder and build the docker image in local *(This can be used to test the catkin make if you dont' have ros installed in your system)* 

```docker build -f ./Dockerfile -t px4sim ..```

Then you can run the docker container and launch the ros nodes

```docker run -p 11311:11311 --name px4sim -it px4sim bash ```

Start mavros inside docker

```nohup roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.29:14557" &```

Source the catkin repo

```source /home/user/ckws/devel/setup.bash```

Launch rc controller node

```rosrun rc_controller rc_controller_node```