// c++
#include "multi_cftld_ros/multi_cftld_nodelet_class.h"

// ros
#include <pluginlib/class_list_macros.h>
#include "cv_bridge/cv_bridge.h"

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// CFtld
#include "tld/DetectorCascade.h"

// Misc
#include "multi_cftld_ros/benchmarker.hpp"

namespace multi_cftld_ros
{

MultiCFtldRosNodelet::MultiCFtldRosNodelet()
  : param_tld_cfgfile_(""),
    param_downsample_factor_(1.0),
    do_downsample_(false),
//    tracking_state_(TRACKING_STATE_UNINITED),
//    tld_ptr_(new tld::TLD()),
    track_id_(0)
{
}

void MultiCFtldRosNodelet::UpdateParams()
{
   util::GetParam<bool>(private_nh_, "enable_debug_image", enable_debug_image_, false);
   util::GetParam<std::string>(private_nh_, "tld_config_file", param_tld_cfgfile_, std::string(""));
   util::GetParam<double>(private_nh_, "downsample_factor", param_downsample_factor_, 1.0);
}

bool MultiCFtldRosNodelet::InitCFtldSettings(const std::string &cfg_file_path)
{
  if (!cfg_file_path.empty())
  {
    NODELET_INFO_STREAM("[CFTLD] Config file " << cfg_file_path);
    try
    {
      tld_cfg_.readFile(cfg_file_path.c_str());
    }
    catch (const libconfig::FileIOException &fioex)
    {
      NODELET_ERROR_STREAM("[CFTLD] I/O error while reading config file: " << fioex.what());
      return false;
    }
    catch (const libconfig::ParseException &pex)
    {
      NODELET_ERROR_STREAM("[CFTLD] ConfigFile parse error" << pex.what());
      return false;
    }
    // If cfg file is valid and succesfully parsed, overwrite default settings
    // Not all are supported
    tld_cfg_.lookupValue("acq.startFrame", tld_settings_.m_startFrame);
    tld_cfg_.lookupValue("acq.lastFrame", tld_settings_.m_lastFrame);
    tld_cfg_.lookupValue("detector.useProportionalShift", tld_settings_.m_useProportionalShift);
    tld_cfg_.lookupValue("detector.proportionalShift", tld_settings_.m_proportionalShift);
    tld_cfg_.lookupValue("detector.minScale", tld_settings_.m_minScale);
    tld_cfg_.lookupValue("detector.maxScale", tld_settings_.m_maxScale);
    tld_cfg_.lookupValue("detector.minSize", tld_settings_.m_minSize);
    tld_cfg_.lookupValue("detector.numTrees", tld_settings_.m_numTrees);
    tld_cfg_.lookupValue("detector.numFeatures", tld_settings_.m_numFeatures);
    tld_cfg_.lookupValue("showOutput", tld_settings_.m_showOutput);
    tld_cfg_.lookupValue("trajectory", tld_settings_.m_trajectory);
    tld_cfg_.lookupValue("printResults", tld_settings_.m_printResults);
    tld_cfg_.lookupValue("printTiming", tld_settings_.m_printTiming);
    tld_cfg_.lookupValue("learningEnabled", tld_settings_.m_learningEnabled);
    tld_cfg_.lookupValue("trackerEnabled", tld_settings_.m_trackerEnabled);
    tld_cfg_.lookupValue("detectorEnabled", tld_settings_.m_detectorEnabled);
    tld_cfg_.lookupValue("detector.varianceFilterEnabled", tld_settings_.m_varianceFilterEnabled);
    tld_cfg_.lookupValue("detector.ensembleClassifierEnabled", tld_settings_.m_ensembleClassifierEnabled);
    tld_cfg_.lookupValue("detector.nnClassifierEnabled", tld_settings_.m_nnClassifierEnabled);
    tld_cfg_.lookupValue("useDsstTracker", tld_settings_.m_useDsstTracker);
    tld_cfg_.lookupValue("selectManually", tld_settings_.m_selectManually);
    tld_cfg_.lookupValue("saveDir", tld_settings_.m_outputDir);
    tld_cfg_.lookupValue("threshold", tld_settings_.m_threshold);
    tld_cfg_.lookupValue("showNotConfident", tld_settings_.m_showNotConfident);
    tld_cfg_.lookupValue("showDetections", tld_settings_.m_showDetections);
    tld_cfg_.lookupValue("alternating", tld_settings_.m_alternating);
    tld_cfg_.lookupValue("seed", tld_settings_.m_seed);
  }
  else
  {
    NODELET_WARN("[CFTLD] Path to the config file is empty. Using default settings ....");
  }
}

bool MultiCFtldRosNodelet::InitCFtld(std::shared_ptr<tld::TLD> &tld_ptr)
{
  // Initialize TLD
  util::SetTLDParam("tld.trackerEnabled", tld_ptr->trackerEnabled, tld_settings_.m_trackerEnabled);
  util::SetTLDParam("tld.detectorEnabled", tld_ptr->detectorEnabled, tld_settings_.m_detectorEnabled);
  tld_ptr->init(tld_settings_.m_useDsstTracker);
  util::SetTLDParam("tld.alternating", tld_ptr->alternating, tld_settings_.m_alternating);
  util::SetTLDParam("tld.learningEnabled", tld_ptr->learningEnabled, tld_settings_.m_learningEnabled);
  util::SetTLDParam("tld.seed", tld_ptr->seed, tld_settings_.m_seed);

//  main->showOutput = m_settings.m_showOutput;
//  main->showTrajectory = (m_settings.m_trajectory) ? true : false;
//  main->trajectoryLength = m_settings.m_trajectory;
//  main->printResults = (m_settings.m_printResults.empty()) ? NULL : m_settings.m_printResults.c_str();
//  main->saveDir = (m_settings.m_outputDir.empty()) ? NULL : m_settings.m_outputDir.c_str();
//  main->threshold = m_settings.m_threshold;
//  main->showNotConfident = m_settings.m_showNotConfident;


//  main->selectManually = m_settings.m_selectManually;
//  main->seed = m_settings.m_seed;

  tld::DetectorCascade *detectorCascade = tld_ptr->detectorCascade;
  ROS_ASSERT(detectorCascade);

  util::SetTLDParam("tld.detectorCascade.varianceFilter.enabled", detectorCascade->varianceFilter->enabled, tld_settings_.m_varianceFilterEnabled);
  util::SetTLDParam("tld.detectorCascade.ensembleClassifier.enabled", detectorCascade->ensembleClassifier->enabled, tld_settings_.m_ensembleClassifierEnabled);
  util::SetTLDParam("tld.detectorCascade.nnClassifier.enabled", detectorCascade->nnClassifier->enabled, tld_settings_.m_nnClassifierEnabled);
  util::SetTLDParam("tld.detectorCascade.nnClassifier.thetaTP", detectorCascade->nnClassifier->thetaTP, tld_settings_.m_thetaP);
  util::SetTLDParam("tld.detectorCascade.nnClassifier.thetaFP", detectorCascade->nnClassifier->thetaFP, tld_settings_.m_thetaN);

  // classifier
  util::SetTLDParam("tld.detectorCascade.useShift", detectorCascade->useShift, tld_settings_.m_useProportionalShift);
  util::SetTLDParam("tld.detectorCascade.shift", detectorCascade->shift, tld_settings_.m_proportionalShift);
  util::SetTLDParam("tld.detectorCascade.minScale", detectorCascade->minScale, tld_settings_.m_minScale);
  util::SetTLDParam("tld.detectorCascade.maxScale", detectorCascade->maxScale, tld_settings_.m_maxScale);
  util::SetTLDParam("tld.detectorCascade.minSize", detectorCascade->minSize, tld_settings_.m_minSize);
  util::SetTLDParam("tld.detectorCascade.numTrees", detectorCascade->numTrees, tld_settings_.m_numTrees);
  util::SetTLDParam("tld.detectorCascade.numFeatures", detectorCascade->numFeatures, tld_settings_.m_numFeatures);

  return true;
}

bool MultiCFtldRosNodelet::ResetRequestCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
/*
  NODELET_WARN("[CFTLD] Reset requested");

  if (tld_ptr_)
  {
    NODELET_INFO("[CFTLD] Releasing the tracker");
    tld_ptr_->release();
  }

  tracking_state_ = TRACKING_STATE_UNINITED;
  track_id_ = 0;
*/
  tld_vec_ptr_.clear();
  tracking_states_.clear();
  uids_.clear();  
  class_ids_.clear();

  track_id_ = 0;

  return true;
}

bool MultiCFtldRosNodelet::InitRequestCallback(multi_cftld_ros::Init::Request &req, multi_cftld_ros::Init::Response &res)
{
  NODELET_INFO_STREAM("[CFTLD] Initialize ROI requested [x,y,w,h]: "
                      << req.roi.x_offset << " , "
                      << req.roi.y_offset << " , "
                      << req.roi.width << " , "
                      << req.roi.height);

  if (frame_input_.empty())
  {
    NODELET_ERROR("[CFTLD] No input frame has been received or input frame is empty!");
    return false;
  }

  // Bad API design of TLD
  cv::Rect bb(req.roi.x_offset, req.roi.y_offset,
              req.roi.width,    req.roi.height);


  util::ClampRect(bb, cv::Rect(0, 0, frame_input_.cols, frame_input_.rows));

  if (bb.area() < 10)
  {
    NODELET_ERROR("[CFTLD] The input rect's area is less than 10");
    return false;
  }

  
  std::shared_ptr<tld::TLD> tld_ptr(new tld::TLD());
  if (!InitCFtld(tld_ptr))
  {
    NODELET_ERROR_STREAM("[CFTLD] Initialzing New CFtld failed!");
  }
  tld_ptr->selectObject(frame_input_, &bb);
  tld_vec_ptr_.push_back(tld_ptr);

  multi_cftld_ros::tracking_state_t tracking_state = TRACKING_STATE_INITED;
  tracking_states_.push_back(tracking_state);

  uids_.push_back(track_id_);  
  class_ids_.push_back(req.class_id);

  track_id_++;
  return true;
}

void MultiCFtldRosNodelet::ImageCallback(const sensor_msgs::ImageConstPtr &img_msg_ptr)
{
  util::StepBenchmarker::GetInstance().reset();

//  if (!tld_ptr_)
//  {
//    // This is the only case that we do not publish any tracking data,
//    // this should actually never happen during healthy lifecycle of this nodelet
//    NODELET_FATAL("[CFTLD] Tracker is NULL");
//    return;
//  }
  try
  {
    if (sensor_msgs::image_encodings::isColor(img_msg_ptr->encoding))
    {
      NODELET_WARN_ONCE("[CFTLD] Input image is BGR8");
      frame_input_cvptr_ = cv_bridge::toCvShare(img_msg_ptr, sensor_msgs::image_encodings::BGR8);
    }
    else
    {
      NODELET_WARN_ONCE("[CFTLD] Input image is MONO8");
      frame_input_cvptr_ = cv_bridge::toCvShare(img_msg_ptr, sensor_msgs::image_encodings::MONO8);
    }

    frame_input_ = frame_input_cvptr_->image;

    TICK("[CFTLD] Frame Copy");

    if (do_downsample_)
    {
      cv::resize(frame_input_, frame_input_, cv::Size(0, 0),
                 param_downsample_factor_, param_downsample_factor_, cv::INTER_CUBIC);
      TICK("[CFTLD] Downsample");
    }

    if (frame_input_.channels() == 1)
    {
      cv::cvtColor(frame_input_, frame_input_, cv::COLOR_GRAY2BGR);
      TICK("[CFTLD] Convert to BGR");
    }

    if (enable_debug_image_ && (pub_debug_image_.getNumSubscribers() > 0))
    {
      frame_debug_ = frame_input_.clone();
    }

    multi_cftld_ros::TracksPtr tracks_msg_ptr(new multi_cftld_ros::Tracks);
    tracks_msg_ptr->header = img_msg_ptr->header;
    tracks_msg_ptr->tracks.reserve(tld_vec_ptr_.size());
    for (int i=0; i < tld_vec_ptr_.size(); i++) {
      std::shared_ptr<tld::TLD> &tld_ptr = tld_vec_ptr_[i];
      multi_cftld_ros::Track track_msg;
      track_msg.status = multi_cftld_ros::Track::STATUS_UNKNOWN;
      track_msg.x = 0.;
      track_msg.y = 0.;
      track_msg.width = 0.;
      track_msg.height = 0.;
      track_msg.roi.x_offset = 0;
      track_msg.roi.y_offset = 0;
      track_msg.roi.width = 0;
      track_msg.roi.height = 0;
      track_msg.uid = uids_[i];
      track_msg.class_id = class_ids_[i];
      track_msg.confidence = 0.0f;

      cv::Rect bb(0, 0, 0, 0);
      if (tracking_states_[i] == multi_cftld_ros::tracking_state_t::TRACKING_STATE_UNINITED)
      {
        NODELET_INFO_THROTTLE(10, "[CFTLD] The tracker has not yet been initialized.");
        track_msg.status = multi_cftld_ros::Track::STATUS_UNKNOWN;
      }
      else
      {
        NODELET_DEBUG_STREAM("TLD Tracking Valid: " << tld_ptr->isTrackerValid);
        NODELET_DEBUG_STREAM("TLD Tracking Confidence: " << tld_ptr->currConf);
        NODELET_DEBUG_STREAM("TLD Tracking Enabled: " << tld_ptr->trackerEnabled);
        NODELET_DEBUG_STREAM("TLD Learning Enabled: " << tld_ptr->learningEnabled);
        NODELET_DEBUG_STREAM("TLD Detection Enabled: " << tld_ptr->detectorEnabled);
        NODELET_DEBUG_STREAM("TLD Valid BB: " << (tld_ptr->currBB != NULL));

        tld_ptr->processImage(frame_input_);
        TICK("[CFTLD] TLD processImage");

        if (tld_ptr->currBB && tld_ptr->isTrackerValid)
        {
          // TODO: Move the boundary rect to the class, init it once
          bb = util::ClampRect(*(tld_ptr->currBB), cv::Rect(0, 0, frame_input_.cols - 1, frame_input_.rows - 1));

          track_msg.status = multi_cftld_ros::Track::STATUS_TRACKING;
          track_msg.roi.x_offset = bb.x;
          track_msg.roi.y_offset = bb.y;
          track_msg.roi.width = bb.width;
          track_msg.roi.height = bb.height;

          track_msg.x = (float) (bb.x + bb.width/2) / (float) frame_input_.cols;
          track_msg.y = (float) (bb.y + bb.height/2) / (float) frame_input_.rows;
          track_msg.width = (float) (bb.width) / (float) frame_input_.cols;
          track_msg.height = (float) (bb.height) / (float) frame_input_.rows;

          track_msg.confidence = tld_ptr->currConf;
          NODELET_DEBUG_STREAM("[CFTLD] Tracking: " << bb);
          tracks_msg_ptr->tracks.push_back(track_msg); // TODO: we might move it below to have all the objects, now only detected objects would add to the main message
        }
        else
        {
          // When lost, except for uid, we send default values (0)
          track_msg.status = multi_cftld_ros::Track::STATUS_LOST;
        }
      }

      // Avoid a message with huge number of elements
      // tracks_msg_ptr->tracks.push_back(track_msg);

      if (enable_debug_image_ && (pub_debug_image_.getNumSubscribers() > 0))
      {
        if (tld_ptr && tld_ptr->currBB)
        {
          //cv::rectangle(frame_debug_, *(tld_ptr_->currBB), CV_RGB(100, 0, 0), 2);
          cv::rectangle(frame_debug_, bb, CV_RGB(100, 0, 0), 2);
        }
      }
    }

    pub_tracks_.publish(tracks_msg_ptr);
    TICK("[CFTLD] Publish results");

    if (enable_debug_image_ && (pub_debug_image_.getNumSubscribers() > 0))
    {
      sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(img_msg_ptr->header,
                                                               sensor_msgs::image_encodings::BGR8,
                                                               frame_debug_).toImageMsg();

      pub_debug_image_.publish(debug_img_msg);
      TICK("[CFTLD] Debug Image");
    }

    NODELET_DEBUG_STREAM("Benchmark:\n" << util::StepBenchmarker::GetInstance().getstr());
  }
  catch (const cv_bridge::Exception& e)
  {
    NODELET_ERROR_STREAM("[CFTLD] cv_bridge exception: " << e.what());
  }
  catch (const ros::Exception& e)
  {
    NODELET_ERROR_STREAM("[CFTLD] ROS exception: " << e.what());
  }
  catch (const std::runtime_error& e)
  {
    NODELET_ERROR_STREAM("[CFTLD] runtime exception: " << e.what());
  }
}

void MultiCFtldRosNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  UpdateParams();

  NODELET_INFO_STREAM("[CFTLD] Initializing CFtld from " << param_tld_cfgfile_);
  if (!InitCFtldSettings(param_tld_cfgfile_))
  {
    NODELET_ERROR_STREAM("[CFTLD] Initialzing CFtld failed!");
  }

  it_ptr_ = std::make_shared<image_transport::ImageTransport>(nh_);

  // In Nodelet version, the default transport hint does not use Nodelet's local and
  // private Nodehandles
  image_transport::TransportHints it_th("raw", ros::TransportHints(), private_nh_);
  sub_image_ = it_ptr_->subscribe("image_raw", 1, &MultiCFtldRosNodelet::ImageCallback, this, it_th);

  NODELET_WARN_STREAM("[CFTLD] Image transport in use: " << sub_image_.getTransport());

  if (enable_debug_image_)
  {
    NODELET_INFO("[CFTLD] The debug image is enabled");
    pub_debug_image_ = it_ptr_->advertise("debug_image", 1);
  }

  init_service_ = nh_.advertiseService("init", &MultiCFtldRosNodelet::InitRequestCallback, this);
  reset_service_ = nh_.advertiseService("reset", &MultiCFtldRosNodelet::ResetRequestCallback, this);
  pub_tracks_ = nh_.advertise<multi_cftld_ros::Tracks>("tracks", 10);

  do_downsample_ = param_downsample_factor_ < 1.0 && param_downsample_factor_ > 0.0;
  NODELET_INFO("[CFTLD] MultiCFtldRosNodelet is initialized.");
}

}  // namespace cftld_ros

PLUGINLIB_DECLARE_CLASS(multi_cftld_ros, MultiCFtldRosNodelet, multi_cftld_ros::MultiCFtldRosNodelet, nodelet::Nodelet);
