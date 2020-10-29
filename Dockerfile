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
	curl \
#	libeigen3-dev \
	libglfw3 \
	libglfw3-dev \
	python3.6 \
	python3.6-dev \
	ros-kinetic-cv-bridge \
	ros-kinetic-opencv3 \
  	ros-kinetic-moveit-core \
  	ros-kinetic-moveit-ros-planning \
 	ros-kinetic-moveit-ros-planning-interface \
	wget	

ENV USERNAME=dti_research
RUN useradd --create-home --shell /bin/bash ${USERNAME}
RUN mkdir -p /home/Workspace

WORKDIR /home/Workspace


#################################################################################################################################################################


#####################
### INSTALL CMAKE ###
#####################
# https://apt.kitware.com/
# If you are using a minimal Ubuntu image or a Docker image, 
# you may need to install the following packages:

RUN apt-get update \
	&& apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common \

# Obtain a copy of our signing key:
	&& wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \

# Add the repository to your sources list and update.
# For Ubuntu Xenial Xerus (16.04):
	&& apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main' \
	&& apt-get update \

# Note that if you add the release candidate repository, 
# you will still need to add the main repository as well, 
# as the release candidate repository does not provide production releases on its own.

# As an optional step, we recommend that you also install our 
# kitware-archive-keyring package to ensure that your keyring 
# stays up to date as we rotate our keys. Do the following:
	&& apt-get install kitware-archive-keyring \
	&& rm /etc/apt/trusted.gpg.d/kitware.gpg \

# Now you can install any package from our APT repository. 
# As an example, try installing the cmake package:
	&& apt-get install -y cmake


##################	
### EXTRA DEPS ###
##################
RUN wget https://bootstrap.pypa.io/get-pip.py \
	&& python3.6 get-pip.py \
	&& rm get-pip.py \
	&& pip install catkin_pkg \
	&& pip install "pybind11[global]" \
	&& pip install pyyaml \
	&& pip install empy \
	&& wget https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.gz \
	&& tar -xzf eigen-3.3.8.tar.gz \
	&& cd eigen-3.3.8 \
	&& mkdir build && cd build \
	&& cmake .. && make install



	
#########################
### INSTALL LIBFRANKA ###
#########################
RUN git clone --recursive https://github.com/frankaemika/libfranka.git \
	&& cd libfranka \
	&& git checkout 0.7.1 \
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
	&& cd src/franka_ros \
	&& git checkout 0.7.1 && cd ../../ \
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
	&& rosdep install -r --from-paths src --ignore-src --rosdistro=kinetic -y \
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
#&& echo "export PYTHONPATH=$PYTHONPATH:/home/Workspace/catkin_ws/devel/bin_picking/scripts" >> ~/.bashrc

	

###################
### ENSENSO SDK ###
###################
RUN cd .. && mkdir Downloads && cd Downloads \
	&& wget -q --show-progress --progress=bar:force https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.3.1536-x64.deb \
	&& dpkg -i download\?files\=ensenso-sdk-2.3.1536-x64.deb 
	
	

