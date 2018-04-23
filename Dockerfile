FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu14.04

MAINTAINER Arturo Gomez Chavez "a.gomezchavez@jacobs-university.de"

# setup environment
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV ROS_DISTRO indigo

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
RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y \
	ros-indigo-desktop \
	&& rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

# install some Pinax ROS dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-indigo-cv-bridge \
    ros-indigo-image-geometry \
    ros-indigo-image-transport \
    ros-indigo-gsll \
    && rm -rf /var/lib/apt/lists/*

# install ceres library for camodocal
RUN apt-get update
RUN wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
RUN tar zxf ceres-solver-1.14.0.tar.gz

# install necessary packages for OpenCV/Ceres/CamoDocal
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
	libtiff4-dev \
	libv4l-dev \
	libatlas-base-dev \
	gfortran \
	git \
	cmake3 \
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
# copy script to install OpenCV in a virtual environment
COPY make_opencv.sh /root
RUN chmod a+x $HOME/make_opencv.sh 
COPY setup_bashrc.sh /root
RUN chmod a+x $HOME/setup_bashrc.sh
RUN /bin/bash -c ". $HOME/setup_bashrc.sh"
RUN /bin/bash -c "cd opencv && . $HOME/make_opencv.sh"

# install camodocal
RUN git clone https://github.com/hengli/camodocal.git
RUN mkdir -p camodocal/build && cd camodocal/build && cmake -DCMAKE_BUILD_TYPE=Release ..
RUN cd camodocal/build && make -j2
RUN cd camodocal/build && make install

# libdc1394 does not work in Docker, so disable it (we anyways don't need it in here)
RUN ln /dev/null /dev/raw1394
    
# create initial workspace
RUN mkdir -p $HOME/pinax/src
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN /bin/bash -c ". /opt/ros/indigo/setup.bash; catkin_init_workspace $HOME/pinax/src"
RUN /bin/bash -c ". /opt/ros/indigo/setup.bash; cd $HOME/pinax; catkin_make"
RUN /bin/bash -c "echo source $HOME/pinax/devel/setup.bash >> $HOME/.bashrc"

# copy and build PinAx code
COPY src /root/pinax/src
RUN /bin/bash -c ". $HOME/pinax/devel/setup.bash && cd pinax && catkin_make"

# provide startup command
#CMD /bin/bash -c ". $HOME/pinax/devel/setup.bash && tail -f /dev/null"
CMD /bin/bash -c ". $HOME/pinax/devel/setup.bash && ./run_pinax_dexrov.sh"
