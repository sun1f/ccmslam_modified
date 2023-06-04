# CC-RGBD-SLAM -- Centralized Collaborative RGB-D SLAM
CC-RGBD-SLAM is a modified verison of [CCM-SLAM](https://github.com/VIS4ROB-lab/ccm_slam), and it supports RGB-D mode.

# Modifiction
The modifiction mainly includes three aspects.
- Add the **RGB-D** mode in the depth branch.
- Use **efficient image transmission** to solve the problem of limited bandwidth.
- Add a **priori map** to the display module of rviz, so that we can clearly observe the clients' position on the map.
## RGB-D mode
The branch **[depth](https://github.com/sun1f/ccmslam_modified/tree/depth)** replaced the monocular mode before to **RGB-D** mode, and the sensor needs to be equipped with a **depth camera**. The client receives the color image and depth image from the camera,and the tracking module basically follows the RGB-D mode of ORB-SLAM2.

## Efficient image transmission
The CCM-SLAM client before receives the original image of the camera sensor, which wil cause excessive communication bandwidth resources and cause the picture to freeze during real-time operation.  
We use the **image_transport** package of ROS to compressed the image topic...

## Display module with priori map
...


