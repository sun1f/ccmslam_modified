# CC-RGBD-SLAM -- Centralized Collaborative RGB-D SLAM
CC-RGBD-SLAM is a modified verison of [CCM-SLAM](https://github.com/VIS4ROB-lab/ccm_slam), and it supports RGB-D mode.

# Modifiction
The modifiction mainly includes three aspects.
- Add the **RGB-D** mode in the depth branch.
- Use **efficient image transmission** to solve the problem of limited bandwidth.
- Add a **priori map** to the display module of rviz, so that we can clearly observe the clients' position on the map.
## RGB-D mode
The branch **[depth](https://github.com/sun1f/ccmslam_modified/tree/depth)** replaced the monocular mode before to **RGB-D** mode, and the sensor needs to be equipped with a **depth camera**. The client receives the color image and depth image from the camera, and the tracking module basically follows the RGB-D mode of ORB-SLAM2.  
It fundamentally **solves the problem of scale uncertainty of the monocular camera**, allowing us to locate on an actual map.

## Efficient image transmission
Former CCM-SLAM client receives the original image of the camera sensor, which wil cause excessive communication bandwidth resources and cause the picture to freeze during real-time operation.  
Therefore, due to the limited bandwidth, efficient image transmission is particularyly important. We use the **image_transport** package of ROS to **compress the image topic**.  
We use Intel RealSense D435i camera as the sensor, the frame rate is set to 30fps, and the image size is **$640\times480$**. We check the original image topic `"/camera/color/image_raw"` and the compressed image topic `"/camera/color/image_raw/compressed"` in ROS, and see that the bandwidth is **28MB/s** and **60KB/s** respectively. Therefore we use compressed image as communication topics for transmission, and **the occupied bandwidth is greatly compressed**.

## Display module with priori map
...


