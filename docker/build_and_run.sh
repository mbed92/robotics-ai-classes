xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="robotics_ai"
ROS_CONTAINER="robotics_ai"
docker build -f Dockerfile -t $ROS_IMAGE ./..

# RUN THE DOCKER CONTAINER
if [[ "$(docker images -q $ROS_IMAGE:latest 2>/dev/null)" == "" ]]; then
  echo "THe image $ROS_IMAGE was not created. Try again."

else
  # INIT SUBMODULES
  git submodules update --init

  # MAP FOLDERS TO ENABLE RVIZ RENDERING INSIDE DOCKER
  # according to 1.3 nvidia-docker2: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
  XAUTH=/tmp/.docker.xauth
  if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
      echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
      touch $XAUTH
    fi
    chmod a+r $XAUTH
  fi

  # RUN THE HARDWARE ACCELERATED CONTAINER
  # SHARED_VOLUME="/robotics_ai/:/catkin_ws/"
  docker run -it \
    --privileged \
    --gpus all \
    --shm-size=16g \
    --ulimit memlock=-1 \
    --memory-swap=-1 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --env="QT_GRAPHICSSYSTEM=native" \
    --env="LD_LIBRARY_PATH=/usr/hostLib64" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume="/usr/lib64:/usr/hostLib64" \
    --network=host \
    --name=$ROS_CONTAINER \
    $ROS_IMAGE \
    bash
fi
