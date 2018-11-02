#include "image_viewer.h"

image_viewer::image_viewer():
	nh_("~"), it_(nh_)
{
	ROS_INFO("Init image_viewer");
	front_image_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &image_viewer::FrontImage, this);
	Odom_nao_ = nh_.subscribe("/imu", 10, &image_viewer::nao_odom, this);
	pub = it_.advertise("oswualdo/image", 1); 
	
	K[0][0]=274.139508945831; // longitud focal fx= 274.139508945831
	K[0][1]=0;
	K[0][2]=141.184472810944;  //centro optico cx=141.184472810944
	K[1][0]=0;
	K[1][1]=275.741846757374;  // longitud focal fy=275.741846757374
	K[1][2]=106.693773654172;  //centro optico cy= 106.693773654172
	K[2][0]=0;
	K[2][1]=0;
	K[2][2]=1;
		
	Kin=K.inverse();
	
	/*
	 image_width: 640	image_height: 480	camera_name: nao_camera_src0_res2
	camera_matrix:  data: [556.845054830986, 0, 309.366895338178, 0, 555.898679730161, 230.592233628776, 0, 0, 1]
  * 
  * image_width: 320	image_height: 240	camera_name: nao_camera_src0_res1
	camera_matrix:  data: [274.139508945831, 0, 141.184472810944, 0, 275.741846757374, 106.693773654172, 0, 0, 1]
  * 
  * image_width: 160	image_height: 120	camera_name: nao_camera_src0_res0
	camera_matrix:  data: [139.424539568966, 0, 76.9073669920582, 0, 139.25542782325, 59.5554242026743, 0, 0, 1] 
	 */
	  
	//matriz de prueba
	prueba[0][0]=1; 
	prueba[0][1]=0;
	prueba[0][2]=0; 
	prueba[1][0]=0;
	prueba[1][1]=1;  
	prueba[1][2]=0;  
	prueba[2][0]=0;
	prueba[2][1]=0;
	prueba[2][2]=1;
}

image_viewer::~image_viewer(){}

void image_viewer::FrontImage(const sensor_msgs::ImageConstPtr &msg)
{	
	try
	{
		cv_ptr_1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	front_image = cv_ptr_1->image;
	
	//escala de grises
	Mat gris, imagen_final=Mat::zeros(front_image.rows, front_image.cols, CV_8UC1);
	cvtColor(front_image, gris, CV_BGR2GRAY);
	imshow( "Escala de grises", gris );
	//H=prueba;
	H=K*R*Kin;
	
	printf("\n matrix Homografía\n");
	for(int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			printf("%f ",H[i][j]);
		}
		printf("\n");
	}
	printf("\n");

	for(int i=0;i<gris.rows;i++)
	{
		for(int j=0;j<gris.cols;j++)
		{
			//multiplicación [xr,yr,1]'= H*[x,y,1]'
			Ir[0]=(H[0][0]*i)+(H[0][1]*j)+(H[0][2]*1);
			Ir[1]=(H[1][0]*i)+(H[1][1]*j)+(H[1][2]*1);
			Ir[2]=(H[2][0]*i)+(H[2][1]*j)+(H[2][2]*1);
			
			//evitar que salga del rango permitido
			if(Ir[0]<0) Ir[0]=0;
			if(Ir[1]<0) Ir[1]=0;
			if(Ir[0]>240) Ir[0]=239;
			if(Ir[1]>320) Ir[1]=319;
			
			// Ir[xr,yr]=I[x,y]
			imagen_final.at<uchar>((int)Ir[0],(int)Ir[1])=(int)gris.at<uchar>(i,j);
		}
	}
	//imshow( "Imagen geoestabilizada", imagen_final ); //visualizar imagen rectificada
	
//##### publicando imagen final ######
//enviar imagen a color
	//enviar=cv_bridge::CvImage(std_msgs::Header(), "bgr8", front_image).toImageMsg(); 
	//pub.publish(enviar);

//publicar imagen en escala de grises
	cv_ptr_1->image = imagen_final;
    cv_ptr_1->encoding = "mono8"; 
    pub.publish(cv_ptr_1->toImageMsg());
	
	waitKey(1);
		
}
void image_viewer::nao_odom(const sensor_msgs::Imu::ConstPtr &odom)
{	
	odom_nao_x = odom->orientation.x;
	odom_nao_y = odom->orientation.y;
	odom_nao_z = odom->orientation.z;
	odom_nao_w = odom->orientation.w;
	
	gyrox=odom->angular_velocity.x; //get data gyroscope
	gyroy=odom->angular_velocity.y;
	gyroz=odom->angular_velocity.z;
	
	accx=odom->linear_acceleration.x; //get data accelerometer	
	accy=odom->linear_acceleration.y;
	accz=odom->linear_acceleration.z;
	
	//ROS_INFO_STREAM("quaternion:["<<odom_nao_x<<" "<<odom_nao_y<<" "<<odom_nao_z<<" "<<odom_nao_w<<"]");
	tf::Quaternion q1(odom_nao_x,odom_nao_y,odom_nao_z,odom_nao_w);
	tf::Matrix3x3 m(q1);
	m.getRPY(roll, pitch, yaw);
	
	

	//prueba filtro acelerometro
	racc=atan2(-accy,-accz);
	pacc=atan2(accx,sqrt(pow(accy,2)+pow(accz,2)));
	
	float alpha=0.5;
	float dt=0.033333333;
	rollf=(1-alpha)*(rollan+rollf*dt)+alpha*racc; //filtro con valores de acelerometro
	pitchf=(1-alpha)*(pitchan+pitchf*dt)+alpha*pacc;
	yawf=(1-alpha)*(yawan+yawf*dt)+alpha*yaw;
	//fin prueba fitro acelerometro
	
	
	/*
	float alpha=0.9;
	float dt=0.033333333;
	rollf=(1-alpha)*(rollan+rollf*dt)+alpha*roll; //filtro con valores obtenidos de imu
	pitchf=(1-alpha)*(pitchan+pitchf*dt)+alpha*pitch;
	yawf=(1-alpha)*(yawan+yawf*dt)+alpha*yaw;
	*/
	//R.setRPY(roll,pitch,yaw);	// sin filtro se ve mejor
	R.setRPY(rollf,pitchf,yawf);	// con filtro
	

	rollan=rollf;
	pitchan=pitchf;
	yawan=yawf;
}
