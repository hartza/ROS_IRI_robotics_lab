
#ifndef CAMCUBE_HH
#define CAMCUBE_HH

#include <pmdsdk2.h>
// ROS include
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
//CAMCUBE
#define SOURCE_PLUGIN "camcube.L32.pap"
#define SOURCE_PARAM ""
#define PROC_PLUGIN "camcubeproc.L32.ppp"
#define PROC_PARAM ""

namespace pmd_camcube
{
 
  using namespace std;
  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }
  
  //! A standard CAMCUBE exception
  DEF_EXCEPTION(Exception, std::runtime_error);
  
  const int CAMCUBE_COLS = 204;
  const int CAMCUBE_ROWS = 204;
  const int CAMCUBE_IMAGES = 3; 

  class PmdCamcube
  {
  public:
    PmdCamcube ();
    ~PmdCamcube ();
    
    int open (int integration_time, bool calibration_on);
    int close();
    /**
    * @param cloud stores the point cloud with XYZ coordinates
    *              Channel r,g,b contains rgb codded depth
    *              Channel intensity contains the normalized intensity [0..1]
    */
//    void readData (pcl::PointCloud<pcl::PointXYZRGB> cloud);
    void readData (sensor_msgs::PointCloud &cloud);
//    void readData (sensor_msgs::PointCloud2 &cloud);
    
//     std::string device_id_;
//     std::string lib_version_;
    /** 
    * sets the limits for the depth to rgb conversion made in readData
    * @param lim depth in cm (default values are 15..50)
    */
    void set_min_depth_limit(int lim);
    void set_max_depth_limit(int lim);
    
    int get_width();
    int get_height();
  private:
    // device identifier
    PMDHandle CC_hnd_;
    PMDDataDescription dd_;
    
    //ImgEntry* imgEntryArray_;
    float * distance_; ///* distance image */
    float * amplitude_; ///* amplitude image */
    float * intensity_; ///* intensity image */
    float * coord_3D_; ///* 3D coordinate image */
    
    int min_depth_limit_;
    int max_depth_limit_;
    
    int width_;
    int height_;
 //   int integration_time_, modulation_freq_;
    
/*    
    int setAutoExposure (bool on);
    int setIntegrationTime (int time);
    int getIntegrationTime ();
    int setModulationFrequency (int freq);
    int getModulationFrequency ();
    int setAmplitudeThreshold (int thresh);
    int getAmplitudeThreshold ();
*/
      
    void SafeCleanup();

    /**
    * given a HSI value returns the corresponding RGB
    * @param H Hue [0..1]
    * @param S Saturation [0..1]
    * @param I Intensity [0..1]
    * TODO: maybe here is not the right place??? something like libUtils....
    */
    void hsi2rgb(float H, float S, float I,float * R, float * G, float * B);
  };
};

#endif
