# pull the image of the long term support distribution humble.
FROM ros:humble

# use app as our working directory
WORKDIR /app

# copy over the python dependencies(currently nothing is used, but it is added for convenience)
COPY requirements.txt /app/requirements.txt

# update apt repos
RUN apt-get update -y

# install pip so python dependencies can be installed
RUN apt-get install python3-pip -y

# install python denpendencies
RUN python3 -m pip install -r /app/requirements.txt

# create the wsg50 workspace
RUN mkdir ./wsg50_ws

# copy over the src files from this docker project to the docker container
COPY ./wsg_ws/src ./wsg50_ws/src

#RUN cd ./wsg50_ws

# set the working directory to the wsg50 workspace
WORKDIR /app/wsg50_ws

# source the ros setup file such that ros commands can be used and build the packages with colcon
RUN . /opt/ros/humble/setup.sh && colcon build

# once the package is built source the install/setup.bash such that the wsg50 packages can be used from the get go
RUN sed --in-place --expression '$isource "/app/wsg50_ws/install/setup.bash"' /ros_entrypoint.sh

# run ros2 run wsg_ctrl wsg to start up the gripper. A launch file version coming soon.
CMD ["ros2", "run", "wsg_ctrl", "wsg"]

