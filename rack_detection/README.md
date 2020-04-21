# tests
To run rack_detection

1. Clone "tests" repository
2. Add .bag file in the "rack_detection" package
3. Run command: roslaunch rack_detection launch.xml
 

# Description

OpenCV is used for detecting rack legs in the image.

The image pipeline is described as follows:

1) Image is converted to hsv image
2) A mask is created to filter out red components in the image
3) Masked image is thresholded
4) Contours are identified from the thresholded image
5) Contours with low area that correspond to image noise are filtered using a threshod area of 400
   pixels
6) Bounding rectangles are drawn over each contour
7) The moment of each bounding rectangle is analysed. Since legs correspond
   to rectangles with small width and large heights, moments of rectangles greater than 0.2 are 
   filtered out. 
8) Coordinates of chair legs can be found by subcribing to /racks_pose topic
