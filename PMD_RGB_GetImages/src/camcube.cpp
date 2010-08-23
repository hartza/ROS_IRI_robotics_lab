#include <signal.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info.h>
#include <tf/transform_listener.h>

#include "camcube.h"

/** @file

@brief camcube device driver


@par Advertises

 - \b distance/image_raw topic (sensor_msgs/Image) raw 2D camera image
   corresponding to measured range depth

 - \b intensity/image_raw topic (sensor_msgs/Image) raw 2D camera
   images corresponding to measured intensity (time synched with
   distance)

 - \b confidence/image_raw topic (sensor_msgs/Image) raw 2D camera
   images corresponding to measurement confidence (time synched with
   distance)

 - \b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each timestep.

 
@par Subscribes

 - None


@par Parameters

- \b frame_id : @b [string] camera frame of reference (Default: device
     node name)

- \b auto_exposure : @b [int] Whether to turn on auto exposure or
  not. 0 == off; > 0 == on; < 0 == use current settings. (Default: 1
  (on))

- \b integration_time : @b [int] Set integration time [SR3k:
  (integration_time+1)*0.200 ms; Sr 0.300ms+(integration_time)*0.100
  ms].  value < 0 results in no change to current settings.  Note:
  auto exposure adapts integration time online, so it is advisabel not
  to set this if using auto exposure.

- \b modulation_freq : @b [int] Set modulation frequency.  (Default:
  no value -- uses factory settings).  value < 0 results in no change
  to current settings.
  The devices employ the following values:
                    0  == 40MHz,  SR3k: maximal range 3.75m
                    1  == 30MHz,  SR3k, Sr: maximal range 5m
                    2  == 21MHz,  SR3k: maximal range 7.14m
                    3  == 20MHz,  SR3k: maximal range 7.5m
                    4  == 19MHz,  SR3k: maximal range 7.89m
                    5  == 60MHz,  Sr: maximal range 2.5m
                    6  == 15MHz,  Sr: maximal range 10m
                    7  == 10MHz,  Sr: maximal range 15m
                    8  == 29MHz,  Sr: maximal range 5.17m
                    9  == 31MHz,  Sr: maximal range 4.84m
                    10 == 14.5MHz, Sr: maximal range 10.34m
                    11 == 15.5MHz, Sr: maximal range 9.68m

- \b amp_threshold : @b [int] Setting this value will set all distance
  values to 0 if their amplitude is lower than the amplitude
  threshold. value < 0 results in no change to current settings.

**/

void sigsegv_handler(int sig);

class CamCubeNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_d_, it_i_, it_c_;
  std::string frame_id_;
  std::string camera_name_;
  sensor_msgs::Image image_d_, image_i_, image_c_;
  sensor_msgs::PointCloud cloud_;
  sensor_msgs::CameraInfo cam_info_;

  CameraInfoManager *cinfo_;

  // reconfigurable parameters
  // None right now, all are set once at runtime.


  ros::Publisher info_pub_;
  image_transport::Publisher image_pub_d_;
  image_transport::Publisher image_pub_i_;
  image_transport::Publisher image_pub_c_;
  ros::Publisher cloud_pub_;

  int auto_exposure_;
  int integration_time_;
  int modulation_freq_;
  int amp_threshold_;
 
public:
  static sr::SR* dev_;

  SRNode(const ros::NodeHandle& nh): nh_(nh), it_d_(nh), it_i_(nh), it_c_(nh)
  {
    signal(SIGSEGV, &sigsegv_handler);
    
    getInitParams();

    info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    image_pub_d_ = it_d_.advertise("distance/image_raw", 1);
    image_pub_i_ = it_i_.advertise("intensity/image_raw", 1);
    image_pub_c_ = it_c_.advertise("confidence/image_raw", 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pointcloud_raw", 1);

    // set reconfigurable parameter defaults (all to automatic)
    // None right now

    // TODO: I dislike opening the device in a constructor
    try
      {
        dev_ = new sr::SR();
        if (dev_->open(auto_exposure_, integration_time_, 
		       modulation_freq_, amp_threshold_) == 0)
          {
            ROS_INFO_STREAM("[" << camera_name_ << "] Connected to device with ID: "
                            << dev_->device_id_);
	    ROS_INFO_STREAM("[" << camera_name_ << "] libmesasr version: " << dev_->lib_version_); 
          }

      }
    catch (sr::Exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown while connecting to the camera:\n"
                         << e.what ());
        nh_.shutdown();
        return;
      }
  } 

  ~SRNode()
  {
    if (dev_)
      {
	dev_->close();
	delete dev_;
	dev_ = NULL;
      }

    delete cinfo_;
  }

  /** get initial parameters (only when node starts). */
  void getInitParams(void)
  {
    if (!nh_.getParam("frame_id", frame_id_))
      {
        // use node name as default frame ID
        frame_id_ = "swissranger";
      }
    ROS_INFO_STREAM("frame ID of camera is " << frame_id_);

    // resolve frame ID using tf_prefix parameter
    std::string tf_prefix = tf::getPrefixParam(nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    frame_id_ = tf::resolve(tf_prefix, frame_id_);

    // default camera name is base name of frame ID
    size_t basepos = frame_id_.find_last_of("/");
    camera_name_ = frame_id_.substr(basepos+1);

    nh_.param("auto_exposure", auto_exposure_, 1);
    nh_.param("integration_time", integration_time_, -1);
    nh_.param("modulation_freq", modulation_freq_, -1);
    nh_.param("amp_threshold", amp_threshold_, -1);


    ROS_INFO_STREAM("[" << camera_name_ << "] , frame ID: " << frame_id_);
    

    cinfo_ = new CameraInfoManager(nh_,camera_name_);

  }

  /** Update reconfigurable parameter.
   *
   * This is done every frame, so we use getParamCached() to avoid
   * unnecessary parameter server requests.
   *
   * @param name ROS parameter private name
   * @param val current parameter value; updated on exit
   * @return true if @a val changed this time
   */
  bool inline updateParam(const std::string &name, int &val)
  {
    bool changed = false;
    int prev = val;
    if (nh_.getParamCached(name, val) && (val != prev))
      changed = true;
    return changed;
  }

  /** Update reconfigurable parameter (for strings). */
  bool inline updateParam(const std::string &name, std::string &val)
  {
    bool changed = false;
    std::string prev = val;
    if (nh_.getParamCached(name, val) && (val != prev))
      changed = true;
    return changed;
  }

  /** Check for changes in reconfigurable parameters. */
  void getParameters()
  {
    static bool first_cycle = true;

//     if (updateParam("brightness", brightness_) || first_cycle)
//       {
//         if (dev_->setBrightness(brightness_) >= 0)
//           {
//             if (brightness_ >= 0)
//               ROS_INFO ("[SRNode] Brightness set to %d", brightness_);
//             else
//               ROS_INFO ("[SRNode] Auto Brightness set");
//           }
//       }

    first_cycle = false;
  }

  /** Main driver loop */
  bool spin()
  {
    while (nh_.ok())
      {
        getParameters();                // check reconfigurable parameters

        // get current CameraInfo data
        cam_info_ = cinfo_->getCameraInfo();
        cloud_.header.frame_id = image_d_.header.frame_id = image_i_.header.frame_id = 
	  image_c_.header.frame_id = cam_info_.header.frame_id = frame_id_;
      
        try
          {
            // Read data from the Camera
            dev_->readData(cloud_,image_d_, image_i_, image_c_);

            cam_info_.header.stamp = image_d_.header.stamp;
            cam_info_.height = image_d_.height;
            cam_info_.width = image_d_.width;
	  
            // Publish it via image_transport
	    info_pub_.publish(cam_info_);
            image_pub_d_.publish(image_d_);
            image_pub_i_.publish(image_i_);
            image_pub_c_.publish(image_c_);
	    cloud_pub_.publish (cloud_);
          }
        catch (sr::Exception& e) {
          ROS_WARN("Exception thrown trying to read data:\n%s",
                   e.what());
          //TODO: shut down and exit?
        }
      
        ros::spinOnce();
      }

    return true;
  }
};

// TODO: figure out a clean way to do this inside SRNode:
sr::SR* SRNode::dev_ = NULL;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
  if (SRNode::dev_)
    {
      SRNode::dev_->close();
    }
}

/** Main entry point */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camcube");
  ros::NodeHandle nh("~");

  CamCubeNode cm(nh);

  cm.spin();
  return 0;
}
