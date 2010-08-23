#include "get_pmd_rgb_images.h"

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
Agafa una imatge de color, llegeix un nuvol de punts i posa color als punts
* Fa servir el calibratge (interns de la RGB i extrinsics) guardats en un fitxer
* Suposa que algú ha engegat la càmera firewire i que publica un missatge
@TODO: fer el mateix amb la camcube

//SORTIDA
* Publica el resultat en un missatge, que es pot visualitzar amb pcl_visualization
* Es pot guardar el resultat activant la variable bSaveFile
*/


  
int main( int argc, char** argv )
{
//TODO: sembla que en Ros la càmera hauria de ser un servei apart!
  bool bSaveFile=false;
  
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
  pmdCC->close();
  return(0);
}

