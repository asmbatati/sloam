SLOAMWS="/media/asmbatati/UbuntuBackup/ros/sloam_ws"
# BAGS_DIR='/media/gnardari/DATA/bags/sloam'
BAGS_DIR='/media/asmbatati/UbuntuBackup/Ag/bags'

CONTAINER_NAME="sloam"
WORKSPACE_DIR="/media/asmbatati/UbuntuBackup/${CONTAINER_NAME}_shared_volume"

# docker run -it \
#     --name="sloam_ros" \
#     --net="host" \
#     --privileged \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --rm \
#     --workdir="/opt/sloam_ws" \
#     --volume="$SLOAMWS:/opt/sloam_ws" \
#     --volume="$HOME:/root" \
#     --volume="/home/$USER/repos:/home/$USER/repos" \
#     --volume="/home/$USER/repos:/home/$USER/repos" \
#     --volume="/home/$USER/ros:/home/$USER/ros" \
#     --volume="$BAGS_DIR:/opt/bags" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     gnardari/sloam:latest \
#     bash

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
    --volume="/home/$USER/repos:/home/$USER/repos" \
    --volume="/run/user/1000/gdm/Xauthority:/run/user/1000/gdm/Xauthority:ro" \
    gnardari/sloam:runtime \
    bash
