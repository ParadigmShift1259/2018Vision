g++ $(pkg-config --libs --cflags opencv) -o VisionProcessing -std=c++11 -L/home/pi/ntcore-4.0.0/build/libs/ntcore/shared -L/home/pi/wpiutil/build/libs/wpiutil/shared -lraspicam -lraspicam_cv -lopencv_core -lntcore -lwpiutil -I/home/pi/wpiutil/src/main/native/include -I/home/pi/ntcore-4.0.0/src/main/native/include -I/home/pi/raspicam-0.1.6/src VisionProcessing.cpp
#sudo /sbin/ldconfig -v
#export ntcore-4.0.0/build/libs/ntcore/shared
#$LD_LIBRARY_PATH = /usr/local/lib
#LD_LIBRARY_PATH = $LD_LIBRARY_PATH:/shared/
#export LD_LIBRARY_PATH
