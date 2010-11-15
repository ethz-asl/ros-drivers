ROS driver for Microsoft Kinect depth sensor, VGA camera, accelerometer and tilt motor.

This driver uses libfreenect. You have to use my patched version, whose git is available at:
	http://github.com/stephanemagnenat/libfreenect

You currently have to manually setup libfreenect using ccmake or cmake-gui in the `build/`
directory. I will provide a package for libfreenect in the futur.
