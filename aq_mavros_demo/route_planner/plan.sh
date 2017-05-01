#!/bin/sh

DIR_BIN=`dirname $(readlink -f $0)`
cd $DIR_BIN

cp waypoints.txt ~/.ros/waypoints.txt
