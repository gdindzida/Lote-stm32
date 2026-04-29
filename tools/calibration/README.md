docker build -t kalibr . | tee build.log

install:
-  apt update && apt install -y python3-wxgtk4.0
- apt update && apt install -y python3-igraph
- apt install -y python3-rosdep


docker run -it --rm \                                                                                                     
  -v ~/workspace/projects/insane-dataset:/data \
  kalibr

source /opt/ros/noetic/setup.bash
source /kalibr_ws/devel/setup.bash
export PATH=/kalibr_ws/devel/lib/kalibr:$PATH

sudo rosdep init
rosdep update

cd /data/insane_nav_cam_klu1_raw/intrinsic

python tools/calibration/rectify_and_crop.py --input /home/mdzida/workspace/projects/insane-dataset/indoor_1_nav_cam/img --output /home/mdzida/workspace/projects/insane-dataset/indoor_1_nav_cam/rectified_img --camchain /home/mdzida/workspace/projects/insane-dataset/insane_nav_cam_klu1_raw/intrinsic/intrinsic-camchain.yaml --size 96 --crop 512
