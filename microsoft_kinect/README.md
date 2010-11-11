ROS driver for Microsoft Kinect depth sensor and VGA camera.

This driver uses libfreenect, which is under heavy development.
There are some patches against libfreenect's git in the `libfreenect-patches/` directory.
Please apply these patches to libfreenect before compiling this driver.

You currently have to manually setup libfreenect using ccmake or cmake-gui in the `build/`
directory. I will provide a package for libfreenect when it will be more stable.
