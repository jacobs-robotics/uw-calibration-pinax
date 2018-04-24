FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu16.04

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
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
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
    && rm -rf /var/lib/apt/lists/*
RUN cd && wget https://bootstrap.pypa.io/get-pip.py && python get-pip.py 
RUN pip install virtualenv virtualenvwrapper

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
RUN cd opencv && git checkout 3.4
RUN cd opencv_contrib && git checkout 3.4
# clone camodocal
RUN git clone https://github.com/hengli/camodocal.git
# copy script to install OpenCV in a virtual environment
COPY make_opencv_camodocal.sh /root
RUN chmod a+x $HOME/make_opencv_camodocal.sh 
COPY setup_bashrc.sh /root
RUN chmod a+x $HOME/setup_bashrc.sh
RUN /bin/bash -c ". $HOME/setup_bashrc.sh"
RUN /bin/bash -c ". $HOME/make_opencv_camodocal.sh"

# libdc1394 does not work in Docker, so disable it (we anyways don't need it in here)
RUN ln /dev/null /dev/raw1394
    
# create initial workspace
RUN mkdir -p $HOME/pinax/src
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash; catkin_init_workspace $HOME/pinax/src"

RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash; cd $HOME/pinax; catkin_make"
RUN /bin/bash -c "echo source $HOME/pinax/devel/setup.bash >> $HOME/.bashrc"

# copy and build PinAx code
COPY src /root/pinax/src
#RUN /bin/bash -c ". $HOME/pinax/devel/setup.bash && cd pinax && catkin_make"

# provide startup command
#CMD /bin/bash -c ". $HOME/pinax/devel/setup.bash && tail -f /dev/null"
#CMD /bin/bash -c ". $HOME/pinax/devel/setup.bash && ./run_pinax_dexrov.sh"
