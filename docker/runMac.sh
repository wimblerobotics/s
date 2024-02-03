docker run \
    -it \
    -v /dev/:/dev/ \
    -e "DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/snd \
    -e ALSADEV=hw:2,0 \
    -v /home/ros/s_ws:/home/ros/s_ws \
    --workdir /home/ros/s_ws \
    --user 1000:1000 \
    macros

#docker run -it --device /dev/snd -e ALSADEV=hw:2,0 -v /dev/:/dev/ -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/ros/s_ws:/home/ros/s_ws --workdir /home/ros/s_ws  --user 1000:1000 macros