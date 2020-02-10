FROM ubuntu:16.04

MAINTAINER Arturo Gomez Chavez "a.gomezchavez@jacobs-university.de"

# setup environment
RUN apt-get clean && apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV ROS_DISTRO kinetic

# create user folders
RUN mkdir -p $HOME/input_data
RUN mkdir -p $HOME/output_data

# upgrade existing packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# install some generic dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# install ROS
RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y \
	ros-kinetic-desktop \
	&& rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

# install some Pinax ROS dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-kinetic-cv-bridge \
    ros-kinetic-image-geometry \
    ros-kinetic-image-transport \
	libgsl-dev \
    && rm -rf /var/lib/apt/lists/*

# install necessary packages for OpenCV/Ceres/CamoDocal
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
	libtiff5-dev \
	libv4l-dev \
	libatlas-base-dev \
	gfortran \
	git \
	cmake \
    libblas-dev \
	libgoogle-glog-dev \
	libsuitesparse-dev \ 
	libgtk2.0-dev \
    && rm -rf /var/lib/apt/lists/*
RUN cd && wget https://bootstrap.pypa.io/get-pip.py && python get-pip.py 

# install ceres library for camodocal
WORKDIR /root
RUN wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
RUN tar zxf ceres-solver-1.14.0.tar.gz
RUN mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver-1.14.0
RUN cd ceres-bin && make -j2
RUN cd ceres-bin && make install 

# install OpenCV
RUN git clone https://github.com/opencv/opencv.git
RUN git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv && git checkout 3.4.7
RUN cd opencv_contrib && git checkout 3.4.7
# clone camodocal
RUN git clone https://github.com/jacobs-robotics/camodocal.git
# copy script to install OpenCV and camodocal
COPY make_opencv_camodocal.sh /root
RUN chmod a+x $HOME/make_opencv_camodocal.sh 
RUN /bin/bash -c ". $HOME/make_opencv_camodocal.sh"

# libdc1394 does not work in Docker, so disable it (we anyways don't need it in here)
RUN ln /dev/null /dev/raw1394 && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get remove -q -y libgtk2.0-dev
#Manual fix to prevent GTK errors
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends libgtk2.0-dev
    
# create initial workspace
RUN mkdir -p $HOME/pinax/src
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash && cd $HOME/pinax/src && catkin_init_workspace"

RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash && cd $HOME/pinax && catkin_make"
RUN /bin/bash -c "echo source $HOME/pinax/devel/setup.bash >> $HOME/.bashrc"

# copy and build PinAx code
COPY src /root/pinax/src
RUN /bin/bash -c ". $HOME/pinax/devel/setup.bash && cd pinax && catkin_make"

# provide startup command
COPY src/run_pinax.sh /root/.
CMD /bin/bash -c ". $HOME/pinax/devel/setup.bash && ./run_pinax.sh"
