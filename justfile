build-px4:
    docker build -f px4-simulator.dockerfile \
    --network=host \
    -t px4-gazebo-garden:latest .

run-px4:
    docker run -it --rm \
        --privileged --net=host \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --name px4-gazebo-garden \
        px4-gazebo-garden bash