FROM ros:kinetic-ros-base AS deplist
COPY . /docker-sample
# Delete all files not named "package.xml" or "requirements.txt"
RUN cd /docker-sample && \
    find . -type f ! -name 'package.xml' ! -name 'requirements.txt' -delete



FROM ros:kinetic-ros-base
# setup Catkin workspace
# RUN apt-get update && apt-get install -y catkin && rm -rf /var/lib/apt/lists/
RUN bash -c "source ros_entrypoint.sh && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    catkin_init_workspace && \
    cd .. && catkin_make"

# Add dependencies
COPY --from=deplist /docker-sample /catkin_ws/src/docker-sample

# Install dependencies
RUN apt-get update && rosdep update && \
    rosdep install --from-paths /catkin_ws/src --ignore-src
    #   && \
    #   rm -rf ~/.ros/rosdep && rm -rf /var/lib/apt/lists/ 


# install package
COPY . /catkin_ws/src/docker-sample

RUN bash -c "source ros_entrypoint.sh && \
    cd /catkin_ws && catkin_make"


#RUN apt-get update && apt-get install -y \
#    ros-kinetic-ros-tutorials \
#    ros-kinetic-common-tutorials \
#    && rm -rf /var/lib/apt/lists/
