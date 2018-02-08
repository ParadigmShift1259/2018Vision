g++ $(pkg-config --libs --cflags opencv) -o VisionProcessing VisionProcessing.cpp -lraspicam -lraspicam_cv -lopencv_core

