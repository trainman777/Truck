cript to delay the launch of a roslaunch file
# 
# Koen Lekkerkerker
# Thu 24 Apr 2014 
#
# Use: ./timed_roslaunch.sh [number of seconds to delay] [rospkg] [roslaunch file]
#

function showHelp(){

if [ "$5" = "-h" ]; then
    showHelp
else 
    sleep $5
    shift
    roslaunch $@
fi
