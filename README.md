# ros_controllers
# ros_controllers
# ros_roverbot

# dependencies
https://github.com/cedricve/raspicam



# Do before compile
- chmod a+x src/roverbot/cfg/Roverbot.cfg
- On pi do this to enable opencv to read the cam
    sudo modprobe bcm2835-v4l2

- git submodule add git@github.com:nadurthi/rosutils.git src/rosutils

- rosrun rqt_reconfigure rqt_reconfigure
