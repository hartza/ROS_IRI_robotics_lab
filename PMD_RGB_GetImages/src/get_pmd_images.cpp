#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include "pcl/ModelCoefficients.h"

//CAMCUBE
#include "camcube.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "sensor_msgs/point_cloud_conversion.h"
//#include <point_cloud_converter/conversion.h>

#include <cmath>
  
/**
* given a HSI value returns the corresponding RGB
* @param H Hue [0..1]
* @param S Saturation [0..1]
* @param I Intensity [0..1]
*/
void hsi2rgb(float H, float S, float I,float * R, float * G, float * B){
  double domainOffset = 0.0;
  if (H<1.0/6.0) { // red domain; green acending
    domainOffset = H;
    *R = I;
    *B = I * (1-S);
    *G = *B + (I-*B)*domainOffset*6;
  }
  else {
    if (H<2.0/6) { // yellow domain; red acending
      domainOffset = H - 1.0/6.0;
      *G = I;
      *B = I * (1-S);
      *R = *G - (I-*B)*domainOffset*6;
    }
    else {
      if (H<3.0/6) { // green domain; blue descending
	domainOffset = H-2.0/6;
	*G = I;
	*R = I * (1-S);
	*B = *R + (I-*R)*domainOffset * 6;
      }
      else {
	if (H<4.0/6) { // cyan domain, green acsending
	  domainOffset = H - 3.0/6;
	  *B = I;
	  *R = I * (1-S);
	  *G = *B - (I-*R) * domainOffset * 6;
	}
	else {
	  if (H<5.0/6) { // blue domain, red ascending
	    domainOffset = H - 4.0/6;
	    *B = I;
	    *G = I * (1-S);
	    *R = *G + (I-*G) * domainOffset * 6;
	  }
	  else { // magenta domain, blue descending
	    domainOffset = H - 5.0/6;
	    *R = I;
	    *G = I * (1-S);
	    *B = *R - (I-*G) * domainOffset * 6;
	  }
	}
      }
    }
  }
}
  
  
int main( int argc, char** argv )
{
//TODO: sembla que en Ros la càmera hauria de ser un servei apart!

  
  pmd_camcube::PmdCamcube * pmdCC = new pmd_camcube::PmdCamcube();
  
  ros::init(argc, argv, "point_clouds");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<sensor_msgs::PointCloud>("visualization_point_cloud", 10);
  ros::Rate r(30);

   ROS_INFO("Open camera connection");
  pmdCC->open(600,true);   

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
    pmdCC->readData(cloud);
    //una mica recargolat no?
//    point_cloud::toMsg (cloudXYZ,cloud2);
    //point_cloud::toMsg (cloudXYZ,cloud2);
    //point_cloud_converter::convert(cloud2,cloud);


    //publish the result
    marker_pub.publish(cloud);
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

    //r.sleep();
    ros::Duration(.1).sleep();
 
  }
  ROS_INFO("Close camera connection");
  pmdCC->close();
  return(0);
}

