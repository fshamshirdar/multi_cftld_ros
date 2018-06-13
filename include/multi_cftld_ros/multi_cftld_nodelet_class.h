#ifndef MULTI_CFTLD_NODELET_CLASS
#define MULTI_CFTLD_NODELET_CLASS

// c++
#include <memory>
#include <string>

// ros
#include <ros/node_handle.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <multi_cftld_ros/Track.h>
#include <multi_cftld_ros/Tracks.h>
#include <multi_cftld_ros/Init.h>
#include <multi_cftld_ros/Reset.h>

// cv
#include <opencv2/core/core.hpp>

// CFtld
#include "tld/TLD.h"
#include "main/Trajectory.h"
#include "main/Settings.h"
#include "cf_tracker.hpp"
#include "libconfig.h++"


namespace multi_cftld_ros
{

enum tracking_state_t
{
  TRACKING_STATE_UNINITED = 0,
  TRACKING_STATE_INITED = 1,
  TRACKING_STATE_LOST = 2,
  TRACKING_STATE_TRACKING = 3,
  TRACKING_STATE_NUM
};


namespace  util
{

// The nodelet version of ROS_* macros complains about getName() not available
template<class T>
static void GetParam(const ros::NodeHandle& nh,
                     const std::string& param_name, T& var, const T& default_value)
{
  nh.param<T>(param_name, var, default_value);
  ROS_INFO_STREAM("[CFTLD] Param " << param_name << " : " << var);
}

template<class T>
static void SetTLDParam(const std::string& caption, T& lhs, const T& rhs)
{
  ROS_INFO_STREAM("[CFTLD] Setting " << caption << " : " << rhs);
  lhs = rhs;
}

template <typename T> inline T clamp (T x, T a, T b)
{
    return ((x) > (a) ? ((x) < (b) ? (x) : (b)) : (a));
}

template <typename T> inline T mapValue(T x, T a, T b, T c, T d)
{
    x = clamp(x, a, b);
    return c + (d - c) * (x - a) / (b - a);
}

template<class T>
inline cv::Rect_<T> ClampRect(const cv::Rect_<T>& rect, const cv::Rect_<T>& boundery) {
  cv::Rect_<T> res = rect;
  res.x = clamp(rect.x, boundery.tl().x, boundery.br().x);
  res.y = clamp(rect.y, boundery.tl().y, boundery.br().y);
  T x2 = clamp(rect.br().x, boundery.tl().x, boundery.br().x);
  T y2 = clamp(rect.br().y, boundery.tl().y, boundery.br().y);
  res.width = x2 - res.x;
  res.height = y2 - res.y;
  return res;
}
}  // namespace util


class MultiCFtldRosNodelet : public nodelet::Nodelet
{
public:
  MultiCFtldRosNodelet();

protected:
  virtual void onInit();
  void ImageCallback(const sensor_msgs::ImageConstPtr& img_msg_ptr);
  bool InitRequestCallback(multi_cftld_ros::Init::Request &req, multi_cftld_ros::Init::Response &res);
  bool ResetRequestCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void UpdateParams();
  bool InitCFtldSettings(const std::string& cfg_file_path);
  bool InitCFtld(std::shared_ptr<tld::TLD> &tld_ptr);

  cv::Mat frame_input_;
  cv::Mat frame_debug_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::shared_ptr<image_transport::ImageTransport> it_ptr_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_debug_image_;
  ros::ServiceServer init_service_; 
  ros::ServiceServer reset_service_; 
  ros::Publisher pub_tracks_;

  // params
  std::string param_tld_cfgfile_;
  double param_downsample_factor_;

  bool do_downsample_;
  bool enable_debug_image_;

  cv_bridge::CvImageConstPtr frame_input_cvptr_;
  cv_bridge::CvImage frame_debug_cvi_;

  // CFtld
  std::vector<multi_cftld_ros::tracking_state_t> tracking_states_;
  std::vector<int> uids_;
  std::vector<int> class_ids_;
  libconfig::Config tld_cfg_;
  tld::Settings tld_settings_;
  std::vector<std::shared_ptr<tld::TLD> > tld_vec_ptr_;

  // Internal
  uint32_t track_id_;  // 0: Unknown (default)

}; // class CFtldRosNodelet
}  // namespace cftld_ros

#endif
