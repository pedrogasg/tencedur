FROM px4io/px4-dev-ros-kinetic

RUN mkdir -p /home/user/ckws/src

RUN chmod 777 /home/user/ckws/src

COPY packages /home/user/ckws/src

RUN apt-get install -y ros-kinetic-angles

#ENV PYTHONPATH "/opt/ros/kinetic/lib/python2.7/dist-packages"

#RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /home/user/ckws/src; /opt/ros/kinetic/bin/catkin_init_workspace"

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /home/user/ckws; /opt/ros/kinetic/bin/catkin_make"
