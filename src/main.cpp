/*
 * main.cpp
 *
 *  Created on: Aug 9, 2012
 *      Author: engelhan
 */

#include "rgbd_utils/calibration_utils.h"

using namespace std;

#include "pinch_recognition/cloud_gmm.h"
#include "pinch_recognition/pinch_detection.h"

#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace enc = sensor_msgs::image_encodings;

const int mean_max_cnt = 20;
int mean_cnt;

Cloud reference, current, changed;
cv::Mat user_mask;
cv::Mat col_img;
ros::Publisher pub_reference, pub_changed;
cv_bridge::CvImagePtr cv_ptr;

int tb_val_min, tb_val_max;

vector<cv::Point> points;

Pinch_detector detector;


/*
 * Find minimum z-value of points and mark all that are not further than x cm from this depth
 *
 *
void z_filter(const std::vector<cv::Point2i>& pts, const Cloud& cloud, float max_dist, std::vector<cv::Point2i>& result){

 result.clear();

 float z_min = 1e6;
 for (uint i=0; i<pts.size(); ++i){
  pcl_Point p = cloud.at(pts[i].x, pts[i].y); if (p.x!=p.x) continue;
  z_min = min(z_min, p.z);
 }

 for (uint i=0; i<pts.size(); ++i){
  pcl_Point p = cloud.at(pts[i].x, pts[i].y); if (p.x!=p.x) continue;

  if (p.z == p.z && p.z < z_min + max_dist)
   result.push_back(pts[i]);
 }

}
*/


void createMaskCallback(int event, int x, int y,int, void* ptr)
{
 cv::Mat* mask = (cv::Mat*) ptr;

 if (event == CV_EVENT_LBUTTONDOWN)
  points.push_back(cv::Point(x,y));

 if (points.size() == 4){
  mask->setTo(0);
  cv::fillConvexPoly(*mask, points, cv::Scalar::all(255));

  cv::namedWindow("user mask");
  cv::imshow("user mask", *mask );
  cv::waitKey(10);

  points.clear();
 }

}


//void tb1_callback(int foo, void* bar){
// cout << foo << endl;
//}

void imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){


 pcl::fromROSMsg(*cloud_ptr, current);



 cv_ptr = cv_bridge::toCvCopy(img_ptr , sensor_msgs::image_encodings::BGR8);
 col_img = cv_ptr->image;

 if (mean_cnt == 0){
  user_mask = cv::Mat(col_img.rows, col_img.cols, CV_8UC1);
  user_mask.setTo(255);
  cv::namedWindow("col");
  cv::setMouseCallback("col", createMaskCallback, &user_mask);

  cv::createTrackbar("min_dist in mm","col", &tb_val_min, 20);
  cv::createTrackbar("max_dist in mm","col", &tb_val_max, 500);



 }

 mean_cnt++;


 if (mean_cnt < mean_max_cnt){
  ROS_INFO("%i", mean_cnt);
  detector.addTrainingFrame(current);
  return;
 }


 if (mean_cnt == mean_max_cnt){

  detector.computeBackground(0.007);



  reference = detector.showBackground(current);

  Cloud::Ptr msg = reference.makeShared();
  msg->header.frame_id =  "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now ();
  pub_reference.publish(msg);

 }

 // todo only once!
 detector.applyMask(user_mask);
 // detector.showMask();


 changed = detector.removeBackground(current, tb_val_min/1000.0, tb_val_max/1000.0);



 cv::namedWindow("foreground");
 cv::imshow("foreground", *detector.getForeground());
 cv::waitKey(10);


 std::vector<cv::Point2f> grasps;
 detector.detectGrasp(grasps);



 for (uint i=0; i<grasps.size(); ++i){
  cv::circle(col_img,grasps[i], 6, cv::Scalar(0,255,255),-1);
 }



 cv::imshow("col", col_img);
 cv::waitKey(10);


 Cloud::Ptr msg = changed.makeShared();
 msg->header.frame_id =  "/openni_rgb_optical_frame";
 msg->header.stamp = ros::Time::now ();
 pub_changed.publish(msg);


}




int main(int argc, char ** argv){


 ros::init(argc, argv,"pinch_detection");

 ros::NodeHandle nh;

 pub_reference = nh.advertise<Cloud>("reference", 1);
 pub_changed = nh.advertise<Cloud>("changed", 1);

 mean_cnt = -1;

 tb_val_min =  10; //mm
 tb_val_max =  200; //mm


 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> policy;
 message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_color", 1);
 message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/rgb/points", 1);
 message_filters::Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
 sync.registerCallback(boost::bind(&imgCloudCB, _1, _2));



 ros::Rate loop_rate(10);
 while ( ros::ok() ) {
  ros::spinOnce();
  loop_rate.sleep();
 }
}






















