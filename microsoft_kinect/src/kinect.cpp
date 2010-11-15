/*
	kinect ROS driver by Stephane Magnenat, ASL, ETHZ
	inspired by the work of:
		Hector Martin
		Alex Trevor
		Ivan Dryanovski
		William Morris
*/
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Vector3.h"
#include <libusb.h>
#include "libfreenect.h"
#include <boost/thread/thread.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <fstream>
#include <sstream>

ros::Publisher pointCloudPub;
ros::Publisher imagePub;
ros::Publisher depthImagePub;
ros::Publisher accPub;
ros::Subscriber tiltSub;


using namespace std;

float t_gamma[2048];

static int depthCounter = 0;

uint8_t rgbImg[640*480*3];

extern "C" void depthimg(uint16_t *buf, int width, int height)
{
	const float horizontalFOVd(57);
	const float verticalFOVd(43);
	
	const float deg2rad(M_PI/180.f);
	const float hf(horizontalFOVd * deg2rad);
	const float vf(verticalFOVd * deg2rad);
	
	// depth image
	sensor_msgs::ImagePtr depthImg(new sensor_msgs::Image);
	depthImg->header.stamp = ros::Time::now();
	depthImg->height = height;
	depthImg->width = width;
	depthImg->encoding = "mono8";
	depthImg->step = width;
	depthImg->data.reserve(width*height);
	uint16_t *srcBuf(buf);
	for (size_t i = 0; i < width*height; ++i)
	{
		depthImg->data.push_back((*srcBuf++) / 8);
	}
	depthImagePub.publish(depthImg);
	
	// point cloud
	sensor_msgs::PointCloudPtr cloud(new sensor_msgs::PointCloud);
	cloud->header.stamp = ros::Time::now();
	cloud->header.frame_id = "/kinect";
	cloud->points.reserve(320*240);
	cloud->channels.resize(1);
	cloud->channels[0].name = "rgb";
	cloud->channels[0].values.reserve(320*240);
	for (int y=0; y<480; y+=2) {
		for (int x=0; x<640; x+=2) {
			const int index(y*640+x);
			const uint16_t rawVal(buf[index]);
			assert (rawVal < 2048);
			if (rawVal < 2047)
			{
				const float dist(t_gamma[rawVal]);
				const float angXimg(-(float(x) * hf)/float(640) + hf/2);
				const float angYimg(-(float(y) * vf)/float(480) + vf/2);
				geometry_msgs::Point32 rosPoint;
				rosPoint.x = 1 * dist;
				rosPoint.y = tan(angXimg) * dist;
				rosPoint.z = tan(angYimg) * dist;
				cloud->points.push_back(rosPoint);
				const uint8_t *rgbPtr = &rgbImg[index*3];
				const unsigned r = *rgbPtr++;
				const unsigned g = *rgbPtr++;
				const unsigned b = *rgbPtr++;
				const unsigned composed = (r << 16) | (g << 8) | b;
				//const unsigned composed = 0xffffff;
				cloud->channels[0].values.push_back(*reinterpret_cast<const float*>(&composed));
			}
		}
	}
	
	// dump
	bool dump;
	ros::param::get("dumpKinect", dump);
	if (dump && ((++depthCounter % 32) == 0))
	{
		ostringstream oss;
		oss << "/tmp/depth-dump-" << (depthCounter / 32) << ".txt";
		ofstream ofs(oss.str().c_str());
		uint16_t *dPtr = buf;
		for (int y=0; y<480; y++) {
			for (int x=0; x<640; x++) {
				ofs << *dPtr++ << "\n";
			}
		}
		
		ostringstream oss2;
		oss2 << "/tmp/color-dump-" << (depthCounter / 32) << ".ppm";
		ofstream ofs2(oss2.str().c_str());
		ofs2 << "P3\n";
		ofs2 << "640 480\n";
		ofs2 << "255\n";
		uint8_t *rgbPtr = rgbImg;
		for (int y=0; y<480; y++) {
			for (int x=0; x<640; x++) {
				const unsigned r = *rgbPtr++;
				const unsigned g = *rgbPtr++;
				const unsigned b = *rgbPtr++;
				ofs2 << r << " " << g << " " << b << "    ";
			}
			ofs2 << "\n";
		}
	}
	
	// show accel
	/*int16_t ax,ay,az;
	acc_get(&ax, &ay, &az);
	ROS_INFO_STREAM("Acceleration: " << ax << ", " << ay << ", " << az);
	*/
	//cerr << "publishing " << cloud->points.size() << " points" << endl;
	
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
	copy(buf, buf + width*height*3, rgbImg);
	imagePub.publish(img);
}

void libusbThread()
{
	ros::Duration(0.01).sleep();
	while(libusb_handle_events(NULL) == 0);
}

void tiltCallback(const std_msgs::Int16& msg)
{
	tilt_set_pos(msg.data);
	ROS_INFO_STREAM("can tilt cmd: " << msg.data);
}

int main(int argc, char **argv)
{
	// static stuff
	for (size_t i=0; i<2048; i++)
	{
		const float k1 = 1.1863;
		const float k2 = 2842.5;
		const float k3 = 0.1236;
		const float depth = k3 * tanf(i/k2 + k1);
		t_gamma[i] = depth;
	}
	
	// libusb stuff
	libusb_init(NULL);
	libusb_device_handle *cam_dev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2ae);
	if (!cam_dev)
	{
		printf("Could not open cam device\n");
		return 1;
	}
	libusb_device_handle *acc_tilt_dev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2b0);
	if (!acc_tilt_dev)
	{
		printf("Could not open acc/tilt device\n");
		return 2;
	}
	
	// ROS node and publisher init
	ros::init(argc, argv, "kinect");
	ros::NodeHandle n;
	pointCloudPub = n.advertise<sensor_msgs::PointCloud>("kinect/cloud", 16);
	imagePub = n.advertise<sensor_msgs::Image>("kinect/image", 16);
	depthImagePub = n.advertise<sensor_msgs::Image>("kinect/depth_image", 16);
	accPub = n.advertise<geometry_msgs::Vector3>("kinect/acc", 16);
	
	// start acquisition
	acc_tilt_init(acc_tilt_dev);
	cams_init(cam_dev, depthimg, rgbimg);
	
	// register to subscriber
	tiltSub = n.subscribe("kinect/tilt", 2, tiltCallback);
	
	boost::thread spin_thread = boost::thread(boost::bind(&libusbThread));
	
	// ros spin
	//ros::spin();
	while (ros::ok())
	{
		ros::spinOnce();
		int16_t ax,ay,az;
		acc_get(&ax, &ay, &az);
		geometry_msgs::Vector3 acc;
		acc.x = float(ax);
		acc.y = float(ay);
		acc.z = float(az);
		accPub.publish(acc);
	}
	
	return 0;
}