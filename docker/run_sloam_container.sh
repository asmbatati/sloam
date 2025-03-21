CONTAINER_NAME="sloam"
SLOAMWS=~/${CONTAINER_NAME}_shared_volume
BAGS_DIR=~/${CONTAINER_NAME}_bags

xhost +local:root # for the lazy and reckless
docker run -it \
    --name="sloam_ros" \
    --net="host" \
    --privileged \
    --rm \
    --gpus="all" \
    --workdir="/opt/sloam_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTHORITY" \
    --env="XAUTHORITY=/run/user/1000/gdm/Xauthority" \
    --volume="$SLOAMWS:/opt/sloam_ws" \
    --volume="$BAGS_DIR:/opt/bags" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    gnardari/sloam:runtime \
    bash
