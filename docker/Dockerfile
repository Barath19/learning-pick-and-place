FROM ros:kinetic
LABEL maintainer="Nicolai Anton Lynnerup nily@dti.dk Lærke Fabricius llfa@dti.dk & Kim Lykke Nørregaard kimn@dti.dk"

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
	curl \
	libglfw3 \
	libglfw3-dev \
	nano \
	python-catkin-tools \
	python3.6 \
	python3.6-dev \
	python3-pip \
	python3-yaml \
	python3-numpy \
	ros-kinetic-cv-bridge \
	ros-kinetic-opencv3 \
  	ros-kinetic-moveit-core \
  	ros-kinetic-moveit-ros-planning \
 	ros-kinetic-moveit-ros-planning-interface \
	ros-kinetic-moveit-simple-controller-manager \
	ros-kinetic-joint-trajectory-controller \
	wget

RUN pip3 install rospkg catkin_pkg

ENV USERNAME=dti_research
RUN useradd --create-home --shell /bin/bash ${USERNAME}
RUN mkdir -p /home/Workspace

WORKDIR /home/Workspace


#################################################################################################################################################################


#####################
### INSTALL CMAKE ###
#####################
# Reference: https://apt.kitware.com/

RUN apt-get update \
	&& apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common \
	&& wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \
	&& apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main' \
	&& apt-get update \
	&& apt-get install kitware-archive-keyring \
	&& rm /etc/apt/trusted.gpg.d/kitware.gpg \
	&& apt-get install -y cmake


##################	
### EXTRA DEPS ###
##################
RUN wget -q --show-progress --progress=bar:force https://bootstrap.pypa.io/get-pip.py \
	&& python3.6 get-pip.py \
	&& rm get-pip.py \
	&& pip install catkin_pkg \
	&& pip install "pybind11[global]" \
	&& pip install pyyaml \
	&& pip install empy \
	&& wget -q --show-progress --progress=bar:force https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz \
	&& tar -xzf eigen-3.3.7.tar.gz \
	&& cd eigen-3.3.7 \
	&& mkdir build && cd build \
	&& cmake .. && make install \
	&& cd ../.. && rm eigen-3.3.8.tar.gz


#######################
### PATCH CV_BRIDGE ###
#######################
RUN cd /usr/lib/x86_64-linux-gnu \
	&& ln -s libboost_python-py35.so libboost_python3.so \
	&& ln -s libboost_python-py35.a libboost_python3.a \
	&& mkdir /home/Workspace/catkin_build_ws && cd /home/Workspace/catkin_build_ws \
	&& catkin config --extend /opt/ros/kinetic \
	&& catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 \
				  -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
				  -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so \
	&& catkin config --install \
	&& mkdir src && cd src \
	&& git clone https://github.com/ros-perception/vision_opencv.git \
	&& cd vision_opencv \
	&& git checkout 1.12.8 \
	&& cd /home/Workspace/catkin_build_ws \
	&& catkin build cv_bridge \
	&& source install/setup.bash --extend

	
#########################
### INSTALL LIBFRANKA ###
#########################
RUN git clone --recursive https://github.com/frankaemika/libfranka.git \
	&& cd libfranka \
	&& git checkout 0.8.0 \
	&& git submodule update \
	&& mkdir build && cd build \
	&& cmake -DCMAKE_BUILD_TYPE=Release .. \
	&& cmake --build .
	

######################################
### INSTALL LIBFRANKA ROS PACKAGES ###
######################################
RUN mkdir -p catkin_ws/src && cd catkin_ws \
	&& /bin/bash -c "source ~/.bashrc" \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_init_workspace src" \
	&& git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros \
	&& cd src/franka_ros \
	&& git checkout 0.7.1 && cd ../../ \
	&& apt-get update \
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
	&& apt-get install -y \
	librealsense2-dkms \
	librealsense2-utils \
	librealsense2-dev  \
	librealsense2-dbg \
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
	&& rosdep install -r --from-paths src --ignore-src --rosdistro=kinetic -y --skip-keys libfranka \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_make clean" \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release" \
	&& /bin/bash -c ". /opt/ros/kinetic/setup.sh; catkin_make install" 
	
RUN echo "source /home/Workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc \
	&& /bin/bash -c "source ~/.bashrc"


##############################
### CLONE THE MAIN PROJECT ###
##############################
RUN mkdir -p catkin_ws/src/learning-pick-and-place
#python3.6 -m pip install -r /home/Workspace/catkin_ws/src/learning-pick-and-place/requirements.txt
#&& echo "export PYTHONPATH=$PYTHONPATH:/home/Workspace/catkin_ws/learning-pick-and-place/scripts" >> ~/.bashrc
# echo "export PYTHONPATH=$PYTHONPATH:/home/Workspace/catkin_ws/devel/lib" >> ~/.bashrc
ENV ROS_MASTER_URI=http://10.224.60.100:11311
ENV ROS_IP=10.224.60.100

RUN pip3 install rospkg

###################
### ENSENSO SDK ###
###################
RUN cd .. && mkdir Downloads && cd Downloads \
	&& wget -q --show-progress --progress=bar:force https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.3.1536-x64.deb \
	&& dpkg -i download\?files\=ensenso-sdk-2.3.1536-x64.deb 


RUN wget https://raw.githubusercontent.com/dti-research/panda_moveit_config/kinetic-devel/config/panda_arm.xacro
	
	

