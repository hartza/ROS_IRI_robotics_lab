#include "get_pmd_rgb_images.h"

#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include "pcl/ModelCoefficients.h"



#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "sensor_msgs/point_cloud_conversion.h"
//#include <point_cloud_converter/conversion.h>
#include "sensor_msgs/Image.h"


#include <cmath>

/**
Agafa una imatge de color, llegeix un nuvol de punts i posa color als punts
* Fa servir el calibratge (interns de la RGB i extrinsics) guardats en un fitxer
* Suposa que algú ha engegat la càmera firewire i que publica un missatge
@TODO: fer el mateix amb la camcube

//SORTIDA
* Publica el resultat en un missatge, que es pot visualitzar amb pcl_visualization
* Es pot guardar el resultat activant la variable bSaveFile
*/


PmdRGB::PmdRGB(ros::NodeHandle &n) : 
  n_(n), it_(n_) 
{ 
  image_sub_ = it_.subscribe("image_topic", 1, &PmdRGB::imageCallback, this);

  pmdCC_ = new pmd_camcube::PmdCamcube();
  integration_time_=600;
  calibration_on_ = true;
  initialised_ = false;
}

PmdRGB::~PmdRGB() {
  if (initialised_)
    pmdCC_->close();
}

void PmdRGB::initialise() {
  pmdCC_->open(integration_time_,calibration_on_);
  initialised_ = true;
}

void PmdRGB::imageCallback (const sensor_msgs::ImageConstPtr& msg_ptr) {
  ROS_INFO("New RGB Image %d x %d",msg_ptr->width,msg_ptr->height);
}
//PmdRGB::getData(sensor_msgs::PointCloud2 &cloud2) {  
//    pmdCC_->readData(cloud); 
//    sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
//}

void PmdRGB::getData(sensor_msgs::PointCloud &cloud) {  
  pmdCC_->readData(cloud);
}

void PmdRGB::setIntegrationTime(int integration_time) {
  integration_time_ = integration_time;
}

void PmdRGB::setCalibration (bool calibration_on) {
  calibration_on_ = calibration_on;
}

int main( int argc, char** argv )
{
  
  
  
  
  //TODO: sembla que en Ros la càmera hauria de ser un servei apart!
  bool bSaveFile=false;
  

  ros::init(argc, argv, "point_clouds");
  ros::NodeHandle n;
//  ros::Publisher marker_pub = n.advertise<sensor_msgs::PointCloud>("visualization_point_cloud", 10);
  ros::Publisher marker_pub = n.advertise<sensor_msgs::PointCloud2>("visualization_point_cloud", 10);
  ros::Rate r(30);

  ROS_INFO("Open camera connection");
  PmdRGB * pPmdRGB = new PmdRGB(n);
  pPmdRGB->initialise();   
  
  //a veure si funciona amb pointclouds
  //pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  //pcl::PCDReader reader;
  //reader.read ("/home/galenya/iri/code/ros/stacks/point_cloud_perception/pcl/test/sac_plane_test.pcd", cloud2);
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::PointCloud cloud;
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
  // Fill in the cloud data
  cloudXYZ.header.frame_id = "/my_frame";
  cloud.header.frame_id = "/my_frame";
  
  int image_counter=0;
  while (ros::ok())
  {
   // pmdCC->readData(cloud);
   pPmdRGB->getData(cloud);
    //una mica recargolat no?
//    point_cloud::toMsg (cloudXYZ,cloud2);
    //point_cloud::toMsg (cloudXYZ,cloud2);
    //point_cloud_converter::convert(cloud2,cloud);
  sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);

  ROS_INFO("New image");
    //publish the result
    marker_pub.publish(cloud2);
    if (bSaveFile) {
	  sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
      // Convert to the templated message type
      //    point_cloud::fromMsg(cloud2,cloudXYZ);
      //    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloudXYZ);
      std::string file_name("test_pcd"); 
      std::stringstream ss;
      ss << image_counter++;
      
      file_name+=ss.str();
      file_name+=".pcd";
      pcl::io::savePCDFileASCII (file_name, cloud2);
      ROS_INFO ("Saved %d data points to %s.", (int)cloud.points.size (),file_name.c_str());
    }
    //r.sleep();
    ros::Duration(.1).sleep();
 
  }
  ROS_INFO("Close camera connection");
  delete pPmdRGB;
  return(0);
}

