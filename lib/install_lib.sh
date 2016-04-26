#!/bin/bash
set -e
set -u

DIR=`pwd`
DST=/home/elwiss/Arduino/libraries/

LIB=QTRSensor 
ln -s $DIR/$LIB $DST/$LIB
