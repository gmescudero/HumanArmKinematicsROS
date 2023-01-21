#! /bin/sh

FILE="elbowAnglesRecording.txt"

echo "\nNEW RECORDING:\n-----------------------------------" >> $FILE
rosservice call /elbowAngles >> $FILE
