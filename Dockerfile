FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu14.04

## Maintainer
MAINTAINER Arturo Gomez Chavez "a.gomezchavez@jacobs-university.de"

## Arguments for users
ARG pinax-user
ARG userid
ARG pinax-user
ARG groupid

## install ros
RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y \
	ros-indigo-desktop \
	&& rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

## set up users and groups
RUN addgroup --gid $groupid $group && \
	adduser --uid $userid --gid $groupid --shell /bin/bash $user && \
	echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$user && \
	chmod 0440 /etc/sudoers.d/$user

# create user folders
RUN mkdir -p /home/$user/input_data
RUN mkdir -p /home/$user/output_data
RUN /bin/bash -c "chown -R $user:$user /home/$user/"


## install necessary packages for OpenCV/Ceres/CamoDocal
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
	libtiff4-dev \
	libv4l-dev \
	libatlas-base-dev \
	gfortran \
	git \
	wget \
	cmake3 \
    libblas-dev \
	libgoogle-glog-dev \
	libsuitesparse-dev \ 
    && rm -rf /var/lib/apt/lists/*
RUN cd /home/$user/ && wget https://bootstrap.pypa.io/get-pip.py && python get-pip.py 
RUN pip install virtualenv virtualenvwrapper

# install ceres library for camodocal
RUN cd /home/$user/ && wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
RUN cd /home/$user/ && tar zxf /home/$user/ceres-solver-1.14.0.tar.gz
RUN mkdir /home/$user/ceres-bin && cd /home/$user/ceres-bin && cmake ../ceres-solver-1.14.0
RUN cd /home/$user/ceres-bin && make -j2
RUN cd /home/$user/ceres-bin && make install 

# clone OpenCV repo
RUN cd /home/$user/ && git clone https://github.com/Itseez/opencv.git
RUN cd /home/$user/ && git clone https://github.com/Itseez/opencv_contrib.git
RUN cd /home/$user/opencv && git checkout 3.4
RUN cd /home/$user/opencv_contrib && git checkout 3.4

# copy script to install OpenCV in a virtual environment
COPY make_opencv.sh /home/$user/.
RUN chmod a+x /home/$user/make_opencv.sh 
COPY setup_bashrc.sh /home/$user/.
RUN chmod a+x /home/$user/setup_bashrc.sh
RUN /bin/bash -c ". /home/$user/setup_bashrc.sh"

# install OpenCV
USER $user
RUN /bin/bash -c ". /home/$user/make_opencv.sh"

# install camodocal
RUN cd /home/$user/ && git clone https://github.com/hengli/camodocal.git
RUN mkdir -p /home/$user/camodocal/build && cd /home/$user/camodocal/build && cmake -DCMAKE_BUILD_TYPE=Release ..
RUN cd /home/$user/camodocal/build && make -j4
RUN make install







