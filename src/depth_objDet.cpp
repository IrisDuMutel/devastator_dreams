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
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void boxesCallback(ros::Publisher& pub, const mydreams::ObjectDetectionBoxes::ConstPtr& boxes_msg, 
                    const sensor_msgs::ImageConstPtr& depth_image_msg ) //::ConstPtr&
{
  ROS_INFO("I heard: ");
  // printf("const char *__restrict __format, ...");
  if (!iszero(boxes_msg->num_boxes))
  {
    // ROS_INFO("I heard: %s", boxes_msg->object[0].c_str());
  
    // float *depth_array = new float[pointcloud_msg->width*pointcloud_msg->height]; // Since it's big I put it in the heap memory!
    // float *depth_array = (float*)malloc(sizeof(float) * depth_image_msg->width*depth_image_msg->height);
    cv_bridge::CvImagePtr cv_ptr;
    struct Point {
    float x;
    float y;
    float z;
    float w;
    };
    Point my_array[boxes_msg->num_boxes - 1];
    Point point;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_image_msg, "");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // ROS_INFO("vertices: %f %f %f %f ", boxes_msg->box_vertices[0], boxes_msg->box_vertices[1], boxes_msg->box_vertices[2], boxes_msg->box_vertices[3]);
    std::vector<double> mean_depth(boxes_msg->num_boxes);
    double min_depth[boxes_msg->num_boxes];
    mydreams::UnityScene boxesDepth_msg;
    for (int i = 0; i < boxes_msg->num_boxes; i++)
    {
      const float ymin_px = std::round(boxes_msg->box_vertices[i*4]*depth_image_msg->height);
      const float xmin_px = std::round(boxes_msg->box_vertices[i*4+1]*depth_image_msg->width);
      const float ymax_px = std::round(boxes_msg->box_vertices[i*4+2]*depth_image_msg->height);
      const float xmax_px = std::round(boxes_msg->box_vertices[i*4+3]*depth_image_msg->width);
      ROS_INFO("vertices: %f %f %f %f ", ymin_px, ymax_px, xmin_px, xmax_px);
      
      Point my_array[boxes_msg->num_boxes - 1];
      Point point;
      
      // // FIXME: sometimes this script throw an opencv error about a matrix with negative dims DONE! a box_vertices was negative and/or out-of-bounds!!
      if (ymax_px > 0 && ymax_px < depth_image_msg->height && ymin_px > 0 && xmax_px > 0 && xmax_px < depth_image_msg->width && xmin_px > 0)
      {
        cv::Mat area_box = cv_ptr->image(cv::Range(ymin_px, ymax_px), cv::Range(xmin_px, xmax_px));   // Creating a sub array of the points within the box
        if (area_box.empty() == false)
        {
          mean_depth[i] = cv::mean(area_box)[0];
          cv::minMaxIdx(area_box, &min_depth[i], NULL, NULL, NULL);
          ROS_INFO("Object: %s at distance %f [min: %f]", boxes_msg->object[i].c_str(), mean_depth[i], min_depth[i]);
          ROS_INFO("i is equal to:  %d", i);
          point.x = ymin_px;
          point.y = xmin_px;
          point.w = ymax_px;
          point.z = xmax_px;
          my_array[i] = point;
          // boxesDepth_msg.ymin_px[i] = ymin_px;
          // boxesDepth_msg.xmin_px[i] = xmin_px;
          // boxesDepth_msg.ymax_px[i] = ymax_px;
          // boxesDepth_msg.xmax_px[i] = xmax_px;
        }
      }
      


    }
    // boxesDepth_msg.header.stamp = ros::Time::now();
    // std::copy(std::begin(boxes_msg->score), std::end(boxes_msg->score), std::begin(boxesDepth_msg.score));
    // std::copy(std::begin(mean_depth), std::end(mean_depth), std::begin(boxesDepth_msg.depth));
    // boxesDepth_msg.num_boxes = boxes_msg->num_boxes;
    // for (size_t i=0; i<boxes_msg->num_boxes; i++)
    // {
        // boxesDepth_msg.object[i] = boxes_msg->object[i];
    // }
    // boxesDepth_msg.height = depth_image_msg->height;
    // boxesDepth_msg.width = depth_image_msg->width;
    // pub.publish(boxesDepth_msg);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "depth");
  ros::NodeHandle n;
  n.getParam("nn_input_size", *nn_input_size);

  image_transport::ImageTransport img_tran(n);
  
  // message_filters::Subscriber<sensor_msgs::Image> boxes_sub(n, "panoramicrgb_img",1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/panoramicd_img",1);
  message_filters::Subscriber<mydreams::ObjectDetectionBoxes> boxes_sub(n, "/DetectionBoxes",1);

  // ImageSubscriber depth_sub(img_tran, "/panoramicd_img", 1);
  
  message_filters::Synchronizer<MySyncPolicy> synchro(MySyncPolicy(1), boxes_sub, depth_sub);
  // message_filters::TimeSynchronizer<object_detection_pico::ObjectDetectionBoxes, PointCloud> synchro(boxes_sub, pointcloud_sub, 1);
  
  ros::Publisher depth_pub = n.advertise<mydreams::UnityScene>("unity_input", 1);
  synchro.registerCallback(boost::bind(&boxesCallback,boost::ref(depth_pub), _1, _2/*, depth_pub*/));
  
  ros::spin();

  return 0;
}