#!/bin/bash
# Visor de camara en vivo en el escritorio del Jetson (gstreamer, sin deps extra).
export DISPLAY=:0
export XAUTHORITY=$(ps -o args= -C Xorg 2>/dev/null | grep -oE "/[^ ]*Xauthority" | head -1)
pkill -f nvarguscamerasrc 2>/dev/null
sleep 1
exec gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1' ! nvvidconv ! videoconvert ! ximagesink sync=false
