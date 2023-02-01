FROM ros:humble

WORKDIR /app

COPY requirements.txt /app/requirements.txt

RUN apt-get update -y

RUN apt-get install python3-pip -y

RUN python3 -m pip install -r /app/requirements.txt

RUN mkdir ./wsg50_ws

COPY ./wsg_ws/src ./wsg50_ws/src

RUN cd ./wsg50_ws

WORKDIR /app/wsg50_ws

RUN . /opt/ros/humble/setup.sh && colcon build

RUN . install/setup.bash

#CMD ["ros2", "run", "--build-type=ament_python", "wsg_ctrl", "wsg"]

#ADD scripts/main.py ./scripts/wsg50_pkg/main.py

#ADD scripts/wsg50.py ./scripts/wsg50.py


#RUN source /opt/ros/humble/setup.bash

#RUN ros2 pkg create --build-type ament_python wsg50_pkg
#CMD [ "ros2", "pkg", "create", "--build-type", "ament_python", "wsg50_pkg"]

#COPY ./scripts/wsg50.py ./wsg50_pkg/wsg50_pkg/wsg50.py
#COPY ./scripts/main.py ./wsg50_pkg/wsg50_pkg/main.py

#CMD ["ros2", "launch", "wsg50_pkg", "wsg50_bringup.launch.py"]

#CMD ["python3", "./scripts/wsg50.py"]

