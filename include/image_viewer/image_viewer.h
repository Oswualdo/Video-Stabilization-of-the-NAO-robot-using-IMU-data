#ifndef IMAGE_VIEWER
#define IMAGE_VIEWER

#include <ros/ros.h>
#include <ros/package.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_msgs/Imu.h"  //odometry

#include <tf/transform_datatypes.h> //matrices
#include <tf/tf.h>

#include <sstream> // for converting the command line parameter to integer


using namespace cv;
using namespace std;

class image_viewer
{

//private:
public:

	ros::NodeHandle nh_;
	
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber front_image_;
  	
  	image_transport::Publisher pub;//publicador de video
	sensor_msgs::ImagePtr enviar;
	//sensor_msgs::ImageConstPtr msg;
	
  
  
	Mat front_image;
	
	cv_bridge::CvImagePtr cv_ptr_1;
	
	//NAO imu
	ros::Subscriber Odom_nao_;
	sensor_msgs::Imu odom;
	
	float odom_nao_x;
	float odom_nao_y;
	float odom_nao_z;
	float odom_nao_w;
	
	float gyrox, gyroy, gyroz; //datos giroscopio
	float accx, accy, accz; //datos acelerometro
	
	double roll;
	double pitch;
	double yaw;
	
	double rollf=0.0, pitchf=0.0,yawf=0.0, rollan=0.0,pitchan=0.0,yawan=0.0,racc,pacc;
	
	tf::Matrix3x3 K, Kin, H,R,prueba,probando; 
	tf::Vector3 Ir;
	
	
public:
	image_viewer();
	~image_viewer();
	
	void FrontImage(const sensor_msgs::ImageConstPtr &msg);
	void nao_odom(const sensor_msgs::Imu::ConstPtr &odom); //nao_imu
};

#endif
