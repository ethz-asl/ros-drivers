#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include <libusb.h>
extern "C" {
#include "libfreenect.h"
}
#include <algorithm>
#include <iostream>

ros::Publisher pointCloudPub;
ros::Publisher imagePub;

using namespace std;

extern "C" void depthimg(uint16_t *buf, int width, int height)
{
	const float horizontalFOVd(57);
	const float verticalFOVd(43);
	
	const float deg2rad(M_PI/180.f);
	const float hf(horizontalFOVd * deg2rad);
	const float vf(verticalFOVd * deg2rad);
	
	sensor_msgs::PointCloudPtr cloud(new sensor_msgs::PointCloud);
	cloud->header.stamp = ros::Time::now();
	cloud->header.frame_id = "/kinect";
	
	cloud->points.reserve(320*240)
	for (int y=0; y<480; y+=2) {
		for (int x=0; x<640; x+=2) {
			const uint16_t val(buf[y*640+x]);
			//if (val < 1280)
			{
				// FIXME: find value for z ratio
				const float dist(float(val)/300.f);
				const float angXimg((float(x) * hf)/float(640) - hf/2);
				const float angYimg(-(float(y) * vf)/float(480) + vf/2);
				// FIXME: find someone fluent with vision to check these hacky transforms 
				geometry_msgs::Point32 rosPoint;
				rosPoint.x = 1 * dist;
				rosPoint.y = tanf(angXimg) * dist;
				rosPoint.z = tanf(angYimg) * dist;
				cloud->points.push_back(rosPoint);
			}
		}
	}
	cerr << "publishing " << cloud->points.size() << " points" << endl;
	
	pointCloudPub.publish(cloud);
}

extern "C" void rgbimg(uint8_t *buf, int width, int height)
{
	sensor_msgs::ImagePtr img(new sensor_msgs::Image);
	img->header.stamp = ros::Time::now();
	img->height = height;
	img->width = width;
	img->encoding = "rgb8";
	img->step = width * 3;
	img->data.reserve(width*height*3);
	copy(buf, buf + width*height*3, back_inserter(img->data));
	imagePub.publish(img);
}

int main(int argc, char **argv)
{
	// libusb stuff
	libusb_init(NULL);
	libusb_device_handle *dev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2ae);
	if (!dev)
	{
		printf("Could not open device\n");
		return 1;
	}
	
	// ROS stuff
	ros::init(argc, argv, "microsoft_kinect");
	ros::NodeHandle n;
	pointCloudPub = n.advertise<sensor_msgs::PointCloud>("cloud", 16);
	imagePub = n.advertise<sensor_msgs::Image>("image", 16);
	
	// start acquisition
	cams_init(dev, depthimg, rgbimg);
	
	// ros spin
	while (ros::ok())
	{
		ros::spinOnce();
		if (libusb_handle_events(NULL) != 0)
			break;
	}
	
	return 0;
}