FROM ros:kinetic
MAINTAINER Lærke Fabricius llfa@teknologisk.dk & Kim Lykke Nørregaard kimn@teknologisk.dk

SHELL ["/bin/bash", "-c"]


########################
### INSTALL PACKAGES ###
########################
RUN apt-get update \
	&& apt-get install -y \
	software-properties-common \
	&& add-apt-repository ppa:deadsnakes/ppa \
	&& apt-get update \
	&& apt-get install -y \
	libeigen3-dev \
	python3.6 \
	ros-kinetic-franka-ros \
	ros-kinetic-libfranka \
	wget \
	&& curl https://bootstrap.pypa.io/get-pip.py | sudo -H python3.6
	

RUN mkdir -p /home/Workspace

WORKDIR /home/Workspace
	
	
#########################
### INSTALL LIBFRANKA ###
#########################
RUN git clone --recursive https://github.com/frankaemika/libfranka.git \
	&& cd libfranka \
	&& git checkout 0.6.0 \
	&& git submodule update \
	&& mkdir build && cd build \
	&& cmake -DCMAKE_BUILD_TYPE=Release .. \
	&& cmake --build .
	

######################################
### INSTALL LIBFRANKA ROS PACKAGES ###
######################################
RUN mkdir -p catkin_ws/src && cd catkin_ws \
	&& echo "source /opt/ros/kinetic/setup.sh" >> .bashrc \
	&& /bin/bash -c "source ~/.bashrc" \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_init_workspace src" \
	&& git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros \
	&& apt-get update \
	&& cd src/franka_ros \
	&& git checkout 0.6.0 && cd ../../ \
	&& rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/Workspace/libfranka/build" \
	&& echo "source /home/Workspace/catkin_ws/devel/setup.sh" >> ~/.bashrc \
	&& /bin/bash -c "source ~/.bashrc"


#########################
### INSTALL REALSENSE ###
#########################
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update \
	# Get lisence key
	&& apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
	&& apt-get install -y software-properties-common \
	# Add the repository 
	&& add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u \
	# Install the librealsense, Development packages & and other needed packages
	&& apt-get install -y librealsense2-dkms \
	&& apt-get install -y librealsense2-utils \
	&& apt-get install -y librealsense2-dev  \
	&& apt-get install -y librealsense2-dbg \
	# Upgrade the local packages 
	&& apt-get update && apt-get --only-upgrade install -y librealsense2-utils librealsense2-dkms

# Create catkin workspace and clone realsense git repository
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
	&& /bin/bash -c "source ~/.bashrc" \
	&& cd catkin_ws/src \
	&& git clone https://github.com/IntelRealSense/realsense-ros.git \
	&& git clone https://github.com/pal-robotics/ddynamic_reconfigure.git \
	&& cd realsense-ros \
	# The latest tag using ROS - and not ROS2
	&& git checkout tags/2.2.17

# Build the workspace from the repository
RUN /bin/bash -c "source ~/.bashrc" \
	&& cd catkin_ws \
	&& rosdep update \
	&& rosdep install -y -r --from-paths src --ignore-src --rosdistro=kinetic -y \
	#&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_init_workspace" \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_make clean" \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release" \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_make install" 
	
RUN echo "source /home/Workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc \
	#&& echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
	&& /bin/bash -c "source ~/.bashrc"


##############################
### CLONE THE MAIN PROJECT ###
##############################
RUN cd catkin_ws/src && mkdir learning-pick-and-place \
# cd src/learning-pick-and-place
#python3.6 -m pip install -r src/learning-pick-and-place/requirements.txt
&& echo "export PYTHONPATH=$PYTHONPATH:/home/Workspace/catkin_ws/devel/bin_picking/scripts" >> ~/.bashrc
	

###################	
### ENSENSO SDK ###
###################
RUN cd .. && mkdir Downloads && cd Downloads \
	&& wget -q --show-progress --progress=bar:force https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.3.1536-x64.deb \
	&& dpkg -i download\?files\=ensenso-sdk-2.3.1536-x64.deb 
	
	


