FROM ros:kinetic-ros-base
# setup Catkin workspace
# RUN apt-get update && apt-get install -y catkin && rm -rf /var/lib/apt/lists/
RUN bash -c "source ros_entrypoint.sh && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    catkin_init_workspace && \
    cd .. && catkin_make"

# install package, with dependencies
COPY . /catkin_ws/src/docker-sample

RUN apt-get update && rosdep update && \
	rosdep install --from-paths /catkin_ws/src --ignore-src && \
	rm -rf ~/.ros/rosdep && rm -rf /var/lib/apt/lists/ 

#RUN apt-get update && apt-get install -y \
#    ros-kinetic-ros-tutorials \
#    ros-kinetic-common-tutorials \
#    && rm -rf /var/lib/apt/lists/
