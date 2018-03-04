FROM ros:kinetic-ros-base AS deplist
COPY . /package
# Delete all files not named "package.xml" or "requirements.txt"
RUN cd /package && \
    find . -type f ! -name 'package.xml' ! -name 'requirements.txt' -delete



FROM ros:kinetic-ros-base
# Upgrade to ros-kinetic-desktop-full
RUN apt-get update && \
	apt-get install -y ros-kinetic-desktop-full && \
	rm -rf /var/lib/apt/lists/

# Install pip
RUN apt-get update && \
	apt-get install -y python-pip && \
	pip install --upgrade pip && \
	rm -rf /var/lib/apt/lists/

# setup Catkin workspace
# RUN apt-get update && apt-get install -y catkin && rm -rf /var/lib/apt/lists/
RUN bash -c "source ros_entrypoint.sh && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    catkin_init_workspace && \
    cd .. && catkin_make && \
    echo 'source /catkin_ws/devel/setup.bash' >> /ros_entrypoint.sh"

# Add dependencies
COPY --from=deplist /package /catkin_ws/src/iarc-2017

# For now, ignore the 3D simulator becacuse it needs such big dependencies
RUN rm -rf \
	/catkin_ws/src/iarc-2017/iarc_sim_3d \
	/catkin_ws/src/iarc-2017/libraries/irobot_create_description

# Install dependencies
RUN bash -c "source ros_entrypoint.sh && \
	apt-get update && rosdep update && \
    rosdep install -y --ignore-src --from-paths /catkin_ws/src && \
    pip install -r catkin_ws/src/**/requirements.txt"
    #   rm -rf ~/.ros/rosdep && rm -rf /var/lib/apt/lists/

# install package
COPY . /catkin_ws/src/iarc-2017

# For now, ignore the 3D simulator becacuse it needs such big dependencies
RUN rm -rf \
	/catkin_ws/src/iarc-2017/iarc_sim_3d \
	/catkin_ws/src/iarc-2017/libraries/irobot_create_description

RUN bash -c "source ros_entrypoint.sh && \
    cd /catkin_ws && catkin_make"
