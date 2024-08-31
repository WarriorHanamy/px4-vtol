#! /usr/bin/bash

PX4_DIR=$PWD

cd $PX4_DIR/gz-aero-plugins

# if build exist
if [ -d "build" ]; then
    echo "gz-aero-plugins build exist"
else
    cmake -Bbuild -S.
    cmake --build build
    cmake --install build
fi



cd $PX4_DIR/dds-agent
# if build exist
if [ -d "build" ]; then
    echo "dds-agent build exist"
    exit
else
    cmake -Bbuild -S.
    cmake --build build
    cmake --install build
fi

sudo ldconfig /usr/local/lib/