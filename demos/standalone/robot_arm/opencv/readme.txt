http://answers.opencv.org/question/25642/how-to-compile-basic-opencv-program-in-c-in-ubuntu/



Use below command to build binary from your source file directory.

g++ -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -o binary  main.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching

where

binary -> The binary file to be created.

main.cpp -> Source file.

Note: Instead for running all these command every time, just create a file named build.sh , put the above command to it and change the file permission to executable by running chmod 777 build.sh, and from next time run this file instead of running the whole command.
