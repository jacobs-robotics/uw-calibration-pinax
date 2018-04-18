FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu14.04

## Maintainer
MAINTAINER Arturo Gomez Chavez "a.gomezchavez@jacobs-university.de"

## Arguments for users
ARG user
ARG userid
ARG group
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



