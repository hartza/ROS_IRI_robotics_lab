
#include <stdint.h>
#include "camcube.h"


//! Macro for throwing an exception with a message
#define SR_EXCEPT(except, msg)					\
  {								\
    char buf[100];						\
    snprintf(buf, 100, "[CamCube::%s]: " msg, __FUNCTION__); \
    throw except(buf);						\
  }

//! Macro for throwing an exception with a message, passing args
#define SR_EXCEPT_ARGS(except, msg, ...)				\
  {									\
    char buf[100];							\
    snprintf(buf, 100, "[CamCube::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf);							\
  }


using namespace pmd_camcube;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Constructor
PmdCamcube::PmdCamcube() : CC_hnd_(NULL)
{    
  min_depth_limit_=15;
  max_depth_limit_=50;    
  
  width_ = CAMCUBE_COLS;
  height_ = CAMCUBE_ROWS;
}

PmdCamcube::~PmdCamcube() 
{
  SafeCleanup();
}


int PmdCamcube::open(int integration_time, bool calibration_on) {
  int res;
  char err[128];
  
  res = pmdOpen (&CC_hnd_, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);
  if (res != PMD_OK)
    {
      pmdGetLastError (0, err, 128);
      SafeCleanup();
      SR_EXCEPT_ARGS(pmd_camcube::Exception, "Could not connect: %s",err);
      return 1;
    }
  
  if (calibration_on)
    pmdSourceCommand (CC_hnd_, 0, 0, "SetLensCalibration On");
  
  res = pmdSetIntegrationTime (CC_hnd_, 0, integration_time);

  res = pmdUpdate (CC_hnd_);
  if (res != PMD_OK)
    {
      SafeCleanup();
      SR_EXCEPT(pmd_camcube::Exception, "Could transfer data");
      return 1;
    }

  res = pmdGetSourceDataDescription (CC_hnd_, &dd_);
  if (res != PMD_OK)
    {
      SafeCleanup();
      SR_EXCEPT(pmd_camcube::Exception, "Could get data description");
      return 1;
    }

  if (dd_.subHeaderType != PMD_IMAGE_DATA)
    {
      SafeCleanup();
      SR_EXCEPT(pmd_camcube::Exception, "Source data is not an image");
      return 1;
    }

  distance_ = new float [dd_.img.numRows * dd_.img.numColumns];
  amplitude_ = new float [dd_.img.numRows * dd_.img.numColumns];
  intensity_ = new float [dd_.img.numRows * dd_.img.numColumns];
  coord_3D_ = new float [dd_.img.numRows * dd_.img.numColumns *3];

  if ((dd_.img.numRows != CAMCUBE_ROWS) || ((dd_.img.numColumns != CAMCUBE_COLS)))
  {
      SafeCleanup();
      SR_EXCEPT_ARGS(pmd_camcube::Exception, "Invalid image size: %d %d\n Expected %d %d",
		     dd_.img.numRows,dd_.img.numColumns,CAMCUBE_ROWS,CAMCUBE_COLS);
      return 1;    
  }
  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Safe Cleanup
void PmdCamcube::SafeCleanup() {
  if (CC_hnd_)
    {
      pmdClose (CC_hnd_);
    }

  if (distance_)
    free(distance_);
  if (amplitude_)
    free(amplitude_);
  if (intensity_)
    free(intensity_);
  if (coord_3D_)
    free(coord_3D_);

  CC_hnd_ = NULL;
  distance_ = NULL;
  amplitude_ = NULL;
  intensity_ = NULL;
  coord_3D_ = NULL;
}



int PmdCamcube::close() {
  if (CC_hnd_)
    if (pmdClose (CC_hnd_)!= PMD_OK)
      ROS_WARN("unable to stop camcube");

  // Free resources
  SafeCleanup();

  return 0;
}





// ////////////////////////////////////////////////////////////////////////////////
// // Store an image frame into the 'frame' buffer
  void PmdCamcube::readData (sensor_msgs::PointCloud &cloud) {
//void PmdCamcube::readData(pcl::PointCloud<pcl::PointXYZRGB> cloud) {

  int res;
  
  if (CC_hnd_ == NULL) {
    SR_EXCEPT(pmd_camcube::Exception, "Read attempted on NULL Camcube port!");
    SafeCleanup();
    return;
  }
  double time1 = ros::Time::now().toSec();
  res = pmdUpdate (CC_hnd_); 
  if (res != PMD_OK)
    {
      SR_EXCEPT(pmd_camcube::Exception,"Could not transfer data");
      SafeCleanup();
      return;
    }
  double time2 = ros::Time::now().toSec();
  double timestamp=(time1+time2)/2;
  
  res = pmdGetDistances (CC_hnd_, distance_, dd_.img.numColumns * dd_.img.numRows * sizeof (float));
  if (res != PMD_OK)
    {
      SR_EXCEPT(pmd_camcube::Exception,"Could not get intensity image");
      SafeCleanup();
      return;
    }
  
  res = pmdGet3DCoordinates (CC_hnd_, coord_3D_, dd_.img.numColumns * dd_.img.numRows * sizeof(float)*3);
  if (res != PMD_OK)
    {
      SR_EXCEPT(pmd_camcube::Exception,"Could get distances image");
      SafeCleanup();
      return;
    }
    
  res = pmdGetAmplitudes (CC_hnd_, amplitude_, dd_.img.numColumns * dd_.img.numRows * sizeof(float)*3);
  if (res != PMD_OK)
    {
      SR_EXCEPT(pmd_camcube::Exception,"Could not get amplitude image");
      SafeCleanup();
      return;
    }
    
  res = pmdGetIntensities (CC_hnd_, intensity_, dd_.img.numColumns * dd_.img.numRows * sizeof(float)*3);
  if (res != PMD_OK)
    {
      SR_EXCEPT(pmd_camcube::Exception,"Could not get intensity image");
      SafeCleanup();
      return;
    }

  cloud.header.stamp=ros::Time(timestamp);
  //image_distance.header.stamp=   image_intensity.header.stamp=image_amplitude.header.stamp=ros::Time(timestamp);
  
  size_t cloud_size=dd_.img.numColumns * dd_.img.numRows;
  // Fill in the cloud data
  cloud.points.resize(cloud_size);
    
  float maxIntens=0.0;
  for (unsigned int i=0;i<dd_.img.numRows;i++)
    {
      for (unsigned int j=0;j<dd_.img.numColumns;j++)
      {
	cloud.points[i*(dd_.img.numRows)+j].x= 100*(coord_3D_[0+i*(dd_.img.numRows*3)+j*3]);
	cloud.points[i*(dd_.img.numRows)+j].y= 100*(coord_3D_[1+i*(dd_.img.numRows*3)+j*3]);
	//satura els punts llunyans
	  cloud.points[i*(dd_.img.numRows)+j].z= 100*(coord_3D_[2+i*(dd_.img.numRows*3)+j*3]);
	//save the corresponding intensity value
//	cloud.points[i*(dd_.img.numRows)+j].rgb= intensity_[i*(dd_.img.numRows)+j];
	//get the max for posterior normalization
	if (intensity_[i*(dd_.img.numRows)+j]>maxIntens) 
	  maxIntens=intensity_[i*(dd_.img.numRows)+j];
      }
    }

    //we need a second pass through the point cloud to normalize intensity... :(
    //Escala entre 0.15 i 0.5m
    cloud.channels.resize (4);
    cloud.channels[0].name="r";
    cloud.channels[0].values.resize (cloud_size);
    cloud.channels[1].name="g";
    cloud.channels[1].values.resize (cloud_size);
    cloud.channels[2].name="b";
    cloud.channels[2].values.resize (cloud_size);
    cloud.channels[3].name="intensity";
    cloud.channels[3].values.resize (cloud_size);
    for (int i=0;i<cloud_size;i++)
    {
//       if ((cloud_filtered.points[i].z<15)||(cloudXYZ.points[i].z>50)) {
// 	 cloud.channels[0].values[i]= 0;
// 	 cloud.channels[1].values[i]= 0;
// 	 cloud.channels[2].values[i]= 0;
//       } else {
	  //use the hsi model to get a different color depending on the depth
	  hsi2rgb((cloud.points[i].z-min_depth_limit_)/(max_depth_limit_-min_depth_limit_),1,.5,
		  &(cloud.channels[0].values[i]),
		  &(cloud.channels[1].values[i]),
		  &(cloud.channels[2].values[i]));
//       }
    //normalize the intensity image between [0..1]
      cloud.channels[3].values[i]= intensity_[i]/maxIntens;	
    }
return;
}



// ////////////////////////////////////////////////////////////////////////////////
// int
// PmdCamcube::setAutoExposure (bool on)
// {
//   int res;
//   if (on)
// #ifdef USE_SR4K
//     res = SR_SetAutoExposure (srCam_, 1, 150, 5, 70);
// #else
//   res = SR_SetAutoExposure (srCam_, 2, 255, 10, 45);
// #endif
//   else
//     res = SR_SetAutoExposure (srCam_, 255, 0, 0, 0);
//   return (res);
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// int
// PmdCamcube::setIntegrationTime (int time)
// {
//   // ---[ Set integration time
//   return (SR_SetIntegrationTime (srCam_, time));
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// int
// PmdCamcube::getIntegrationTime ()
// {
//   // ---[ Set integration time
//   return (SR_GetIntegrationTime (srCam_));
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// int
// PmdCamcube::setModulationFrequency (int freq)
// {
//   // ---[ Set modulation frequency
//   return (SR_SetModulationFrequency (srCam_, (ModulationFrq)freq));
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// int
// PmdCamcube::getModulationFrequency ()
// {
//   // ---[ Set modulation frequency
//   return (SR_GetModulationFrequency (srCam_));
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// int
// PmdCamcube::setAmplitudeThreshold (int thresh)
// {
//   // ---[ Set amplitude threshold
//   return (SR_SetAmplitudeThreshold (srCam_, thresh));
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// int
// PmdCamcube::getAmplitudeThreshold ()
// {
//   // ---[ Set amplitude threshold
//   return (SR_GetAmplitudeThreshold (srCam_));
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// // Obtain the device product name
// std::string
// PmdCamcube::getDeviceString ()
// {
//   char *buf = new char[256];
//   int *buflen = new int;
//   SR_GetDeviceString (srCam_, buf, *buflen);
// 
//   // VendorID:0x%04x, ProductID:0x%04x, Manufacturer:'%s', Product:'%s'
//   std::string sensor (buf);
//   std::string::size_type loc = sensor.find ("Product:", 0);
//   if (loc != std::string::npos)
//     {
//       sensor = sensor.substr (loc + 9, *buflen);
//       loc = sensor.find ("'", 0);
//       if (loc != std::string::npos)
// 	sensor = sensor.substr (0, loc);
//     }
//   else
//     sensor = "";
// 
//   delete buflen;
//   delete [] buf;
//   return (sensor);
// }
// 
// ////////////////////////////////////////////////////////////////////////////////
// // Obtain the libMesaSR library version
// std::string
// PmdCamcube::getLibraryVersion ()
// {
//   unsigned short version[4];
//   char buf[80];
//   SR_GetVersion (version);
//   snprintf (buf, sizeof (buf), "%d.%d.%d.%d", version[3], version[2], version[1], version[0]);
//   return (std::string (buf));
// }

    int PmdCamcube::get_width() {
      return width_;
    }
    
    int PmdCamcube::get_height() {
      return height_;
    }


void PmdCamcube::hsi2rgb(float H, float S, float I,float * R, float * G, float * B){
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
  
