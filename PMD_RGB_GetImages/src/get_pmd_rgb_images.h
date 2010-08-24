#ifndef GET_PMD_RGB_IMAGES_H
#define GET_PMD_RGB_IMAGES_H

//CAMCUBE
#include "camcube.h"

#include <ros/ros.h>
#include "image_transport/image_transport.h"
class PmdRGB {
  public:
    PmdRGB(ros::NodeHandle &n);
    ~PmdRGB();
    void initialise();
//    void getData(sensor_msgs::PointCloud2 &cloud2);
    void getData(sensor_msgs::PointCloud &cloud);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
    void setIntegrationTime(int integration_time);
    void setCalibration (bool calibration_on);
  private:
    //to handle 1394 images
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    //to handle camcuble
    pmd_camcube::PmdCamcube * pmdCC_;  
    
    int integration_time_;///600 by default (close objects)
    bool calibration_on_;
    bool initialised_; ///if the camera has been initialised should be cleanly closed
};

#endif