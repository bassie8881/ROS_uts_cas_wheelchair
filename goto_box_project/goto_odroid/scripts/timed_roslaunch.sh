#!/bin/bash

if [ "$1" = "-h" ]; then
    showHelp
else 
    echo "start wait for $1 seconds"
    sleep $1
    echo "end wait for $1 seconds"
    shift
        echo "now running 'roslaunch $@'"
    roslaunch $@
fi
