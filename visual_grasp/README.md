There is a script for using realsense in gazebo to get a pointcloud scene.  
You need install the python dependencies:
|dependencies|
| :---: |
|opencv-python==4.10.0.48|
|open3d==0.13.0|
|cv_bridge==1.16.2|
|pandas==2.0.3|

You can use ```rostopic echo /d435/depth/camera_info``` to get camera intrinsics.
