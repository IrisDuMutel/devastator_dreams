#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
// #include <sensor_msgs/Image.h>
#include <cstdio>
#include <cstring>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h> // From https://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/
#include <algorithm>
#include <boost/concept_check.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <numeric>
#include <vector>
#include "mydreams/ObjectDetectionBoxes.h"
#include "mydreams/UnityScene.h"

#include "message_filters/time_synchronizer.h"
#include <message_filters/synchronizer.h>
#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>


// WHAT IS THIS NODE FOR?
// This node adds to the message coming from CNN_onjDet. 
// The message is read and depth is added to each object,
// and then published for Unity.

// #include <librealsense2/rs.hpp>

int nn_input_size[2];

typedef message_filters::sync_policies::ApproximateTime<mydreams::ObjectDetectionBoxes, sensor_msgs::Image> MySyncPolicy;
typedef image_transport::SubscriberFilter ImageSubscriber; // Using image transport package

// CALLBACK DEFINITION
void boxesCallback(ros::Publisher& pub, const mydreams::ObjectDetectionBoxes::ConstPtr& boxes_msg, 
                    const sensor_msgs::ImageConstPtr& depth_image_msg ) 
{

  // ROS_INFO("I heard: ");
  // // printf("const char *__restrict __format, ...");

  // IF OBJECTS WERE FOUND, THEN ENTER IF
  if (!iszero(boxes_msg->num_boxes))
  {
  

    cv_bridge::CvImagePtr cv_ptr;

    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_image_msg, "");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<double> mean_depth(boxes_msg->num_boxes);
    double min_depth[boxes_msg->num_boxes];
    mydreams::UnityScene boxesDepth_msg;

    // FOR EACH OBJECT DETECTED
    for (int i = 0; i < boxes_msg->num_boxes; i++)
    {
      const uint32_t ymin_px = std::round(boxes_msg->box_vertices[i*4]*depth_image_msg->height);
      const uint32_t xmin_px = std::round(boxes_msg->box_vertices[i*4+1]*depth_image_msg->width);
      const uint32_t ymax_px = std::round(boxes_msg->box_vertices[i*4+2]*depth_image_msg->height);
      const uint32_t xmax_px = std::round(boxes_msg->box_vertices[i*4+3]*depth_image_msg->width);
      ROS_INFO("vertices: %d %d %d %d ", ymin_px, ymax_px, xmin_px, xmax_px);
      
      // FIXME: sometimes this script throw an opencv error about a matrix with negative dims DONE! a box_vertices was negative and/or out-of-bounds!!
      if (ymax_px > 0 && ymax_px < depth_image_msg->height && ymin_px > 0 && xmax_px > 0 && xmax_px < depth_image_msg->width && xmin_px > 0)
      {
        cv::Mat area_box = cv_ptr->image(cv::Range(ymin_px, ymax_px), cv::Range(xmin_px, xmax_px));   // Creating a sub array of the points within the box
        if (area_box.empty() == false)
        {
          mean_depth[i] = cv::mean(area_box)[0];
          cv::minMaxIdx(area_box, &min_depth[i], NULL, NULL, NULL);
          ROS_INFO("Object: %s at distance %f [min: %f]", boxes_msg->object[i].c_str(), mean_depth[i], min_depth[i]);
          boxesDepth_msg.ymin_px.push_back(ymin_px);
          boxesDepth_msg.xmin_px.push_back(xmin_px);
          boxesDepth_msg.ymax_px.push_back(ymax_px);
          boxesDepth_msg.xmax_px.push_back(xmax_px);
        }
      }
    }
    
    // boxesDepth_msg.header.stamp = ros::Time::now();
    for (std::vector<float>::const_iterator it = boxes_msg->score.begin(); it != boxes_msg->score.end(); ++it) {
      boxesDepth_msg.score.push_back(*it);
    }

    for (std::vector<double>::const_iterator it = mean_depth.begin(); it != mean_depth.end(); ++it) {
      boxesDepth_msg.depth.push_back(*it);
    }

    boxesDepth_msg.num_boxes = boxes_msg->num_boxes;
    for (size_t i=0; i<boxes_msg->num_boxes; i++)
    {
        boxesDepth_msg.object.push_back(boxes_msg->object[i]);
    }

    boxesDepth_msg.height = depth_image_msg->height;
    boxesDepth_msg.width = depth_image_msg->width;
    pub.publish(boxesDepth_msg);
  }
}

int main(int argc, char **argv)
{
  // NODE INITIALIZATION
  ros::init(argc, argv, "depth");
  ros::NodeHandle n;

  n.getParam("nn_input_size", *nn_input_size);

  // SUBSCRIBERS
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/panoramicd_img",1);
  message_filters::Subscriber<mydreams::ObjectDetectionBoxes> boxes_sub(n, "/DetectionBoxes",7);


  // PUBLISHERS
  ros::Publisher depth_pub = n.advertise<mydreams::UnityScene>("unity_input", 1);

  // SYNCHORONIZER
  message_filters::Synchronizer<MySyncPolicy> synchro(MySyncPolicy(1), boxes_sub, depth_sub);
  synchro.registerCallback(boost::bind(&boxesCallback,boost::ref(depth_pub), _1, _2/*, depth_pub*/));
  
  // Keep the node spinning
  ros::spin();
  return 0;
}