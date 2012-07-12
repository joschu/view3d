// mostly copied from stereo_view.cpp in image_view package, but modified to arrange windows

/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <highgui.h>


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/CvBridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <X11/Xlib.h>

#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroy(GtkWidget *widget, gpointer data)
{
  ros::shutdown();
}
#endif

namespace enc = sensor_msgs::image_encodings;

void increment(int* value)
{
  ++(*value);
}

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

void getScreenSize(int& width, int& height) {
  Display *display;
  int screen_num;
  unsigned int display_width, display_height;
  char* display_name = NULL;
  char* progname = "progname";

  if ( (display=XOpenDisplay(display_name)) == NULL )

    {
      (void) fprintf( stderr, "%s: cannot connect to X server %s\n",
                      progname, XDisplayName(display_name));
      exit( -1 );
    }


  screen_num = DefaultScreen(display);
  width = DisplayWidth(display, screen_num);
  height = DisplayHeight(display, screen_num);
  XCloseDisplay(display);
}


// Note: StereoView is NOT nodelet-based, as it synchronizes the three streams.
class StereoView
{
private:
  image_transport::SubscriberFilter left_sub_, right_sub_;
  typedef ExactTime<Image, Image> ExactPolicy;
  typedef ApproximateTime<Image, Image> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;
  
  ImageConstPtr last_left_msg_, last_right_msg_;
  cv::Mat last_left_image_, last_right_image_;
  CvBridge left_bridge_, right_bridge_;
  boost::mutex image_mutex_;
  
  boost::format filename_format_;
  int save_count_;

  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_, disp_received_, all_received_;

public:
  StereoView(const std::string& transport)
    : filename_format_(""), save_count_(0),
      left_received_(0), right_received_(0), disp_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    
    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("%s%04i.jpg"));
    filename_format_.parse(format_string);

    // Do GUI window setup
    int flags = CV_WINDOW_NORMAL;
    cv::namedWindow("left", flags);
    cv::namedWindow("right", flags);
    cvSetMouseCallback("left",      &StereoView::mouseCb, this);
    cvSetMouseCallback("right",     &StereoView::mouseCb, this);
#ifdef HAVE_GTK
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("left") ),
                     "destroy", G_CALLBACK(destroy), NULL);
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("right") ),
                     "destroy", G_CALLBACK(destroy), NULL);
#endif
    cvStartWindowThread();

    int screen_width, screen_height;
    getScreenSize(screen_width, screen_height);
    cvMoveWindow("left", 0, 0);
    cvMoveWindow("right", screen_width/2, 0);
    cvResizeWindow("left", screen_width/2, screen_width/2);
    cvResizeWindow("right", screen_width/2, screen_width/2);



    // Resolve topic names
    ros::NodeHandle nh;
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    ROS_INFO("Subscribing to:\n\t* %s\n\t* %s", left_topic.c_str(), right_topic.c_str());

    // Subscribe to three input topics.
    image_transport::ImageTransport it(nh);
    left_sub_.subscribe(it, left_topic, 1, transport);
    right_sub_.subscribe(it, right_topic, 1, transport);

    // Complain every 30s if the topics appear unsynchronized
    left_sub_.registerCallback(boost::bind(increment, &left_received_));
    right_sub_.registerCallback(boost::bind(increment, &right_received_));
    check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0),
                                             boost::bind(&StereoView::checkInputsSynchronized, this));

    // Synchronize input topics. Optionally do approximate synchronization.
    local_nh.param("queue_size", queue_size_, 5);
    bool approx;
    local_nh.param("approximate_sync", approx, true);
    if (approx)
    {
      approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size_),
                                                   left_sub_, right_sub_) );
      approximate_sync_->registerCallback(boost::bind(&StereoView::imageCb, this, _1, _2));
    }
    else
    {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size_),
                                       left_sub_, right_sub_) );
      exact_sync_->registerCallback(boost::bind(&StereoView::imageCb, this, _1, _2));
    }
  }

  ~StereoView()
  {
    cvDestroyAllWindows();
  }

  void imageCb(const ImageConstPtr& left, const ImageConstPtr& right)
  {
    ++all_received_; // For error checking
    
    image_mutex_.lock();

    // May want to view raw bayer data
    if (left->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<Image>(left)->encoding = "mono8";
    if (right->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<Image>(right)->encoding = "mono8";

    // Hang on to image data for sake of mouseCb
    last_left_msg_ = left;
    last_right_msg_ = right;
    try {
      last_left_image_ = left_bridge_.imgMsgToCv(left, "bgr8");
      last_right_image_ = right_bridge_.imgMsgToCv(right, "bgr8");
    }
    catch (CvBridgeException& e) {
      ROS_ERROR("Unable to convert one of '%s' or '%s' to 'bgr8'",
                left->encoding.c_str(), right->encoding.c_str());
    }

    // Must release the mutex before calling cv::imshow, or can deadlock against
    // OpenCV's window mutex.
    image_mutex_.unlock();
    if (!last_left_image_.empty())
      cv::imshow("left", last_left_image_);
    if (!last_right_image_.empty())
      cv::imshow("right", last_right_image_);
  }

  void saveImage(const char* prefix, const cv::Mat& image)
  {
    if (!image.empty()) {
      std::string filename = (filename_format_ % prefix % save_count_).str();
      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save %s image, no data!", prefix);
    }
  }
  
  static void mouseCb(int event, int x, int y, int flags, void* param)
  {
    if (event == CV_EVENT_LBUTTONDOWN)
    {
      ROS_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
      return;
    }
    if (event != CV_EVENT_RBUTTONDOWN)
      return;
    
    StereoView *sv = (StereoView*)param;
    boost::lock_guard<boost::mutex> guard(sv->image_mutex_);

    sv->saveImage("left",  sv->last_left_image_);
    sv->saveImage("right", sv->last_right_image_);
    sv->save_count_++;
  }

  void checkInputsSynchronized()
  {
    int threshold = 3 * all_received_;
    if (left_received_ >= threshold || right_received_ >= threshold) {
      ROS_WARN("[view3d] Low number of synchronized left/right triplets received.\n"
               "Left images received:      %d (topic '%s')\n"
               "Right images received:     %d (topic '%s')\n"
               "Synchronized triplets: %d\n"
               "Possible issues:\n"
               "\t* stereo_image_proc is not running.\n"
               "\t  Does `rosnode info %s` show any connections?\n"
               "\t* The cameras are not synchronized.\n"
               "\t  Try restarting stereo_view with parameter _approximate_sync:=True\n"
               "\t* The network is too slow. One or more images are dropped from each triplet.\n"
               "\t  Try restarting stereo_view, increasing parameter 'queue_size' (currently %d)",
               left_received_, left_sub_.getTopic().c_str(),
               right_received_, right_sub_.getTopic().c_str(),
               all_received_, ros::this_node::getName().c_str(), queue_size_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_view", ros::init_options::AnonymousName);
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun view3d view3d stereo:=narrow_stereo image:=image_color");
  }
  if (ros::names::remap("image") == "/image_raw") {
    ROS_WARN("There is a delay between when the camera drivers publish the raw images and "
             "when stereo_image_proc publishes the computed point cloud. stereo_view "
             "may fail to synchronize these topics without a large queue_size.");
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  StereoView view(transport);
  
  ros::spin();
  return 0;
}
