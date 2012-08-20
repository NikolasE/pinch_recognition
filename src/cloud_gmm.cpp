/*
 * cloud_gmm.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: lengelhan
 */


#include "pinch_recognition/cloud_gmm.h"

using namespace std;

uint Background_substraction::addTrainingFrame(const Cloud& c){

 if (dists.size() == 0){
  means = cv::Mat(c.height, c.width, CV_32FC1);
  std_dev = cv::Mat(c.height, c.width, CV_32FC1);
  reset_helper_images();
 }

 cv::Mat new_dist = cv::Mat(c.height, c.width, CV_32FC1);
 new_dist.setTo(0);

 for (uint x=0; x<c.width; ++x)
  for (uint y=0; y<c.height; ++y){
   pcl_Point p = c.at(x,y);

   // store distance to object if available, -1 else
   new_dist.at<float>(y,x) = (p.x==p.x)?norm(p):-1;
  }

 dists.push_back(new_dist);
 return dists.size();

}


cv::Mat Background_substraction::applyMask(cv::Mat& img){

 if (mask.cols != img.cols){
  ROS_WARN("no background computed!");
  return cv::Mat();
 }

 cv::Mat result;
 img.copyTo(result, mask);
 return result;

}


// untested
Cloud Background_substraction::applyMask(Cloud& current){

 if (uint(mask.cols) != current.width){
  ROS_WARN("no background computed!");
  return Cloud();
 }

 Cloud result;
 result.reserve(current.size());

 for (uint x=0; x<current.width; ++x)
  for (uint y=0; y<current.height; ++y){
   if (mask.at<uchar>(y,x) == 0)
    result.push_back(current.at(x,y));
  }

 return result;

}


Cloud Background_substraction::removeBackground(const Cloud& current, float max_dist, std::vector<cv::Point2i>* valids){

 Cloud result;
// assert(cloud.size() == reference.size());

// for (uint i=0; i<cloud.size(); ++i){

 for (uint x=0; x<current.width; ++x)
  for (uint y=0; y<current.height; ++y){

  pcl_Point c = current.at(x,y);


  if (c.x != c.x) continue;

  if (mask.at<uchar>(y,x) == 0) continue;

  float n = norm(c);
  float mean = means.at<float>(y,x);

  // ROS_INFO("Norm: %f, mean: %f", n, mean);

  if ( n < mean - max_dist){
   result.push_back(c);
   if (valids) valids->push_back(cv::Point2i(x,y));
  }

 }

 return result;

}


bool Background_substraction::computeBackground(float max_std_dev){

 reset_helper_images();

 // if (training_frames_cnt == 0){
 //  ROS_WARN("Background_substraction: No training frames!");
 //  return false;
 // }
 //
 // if (training_frames_cnt == 1){
 //  ROS_WARN("Background_substraction: Only one training frame!");
 //  mean = dist_sum;
 //  return false;
 // }

 if (dists.size() == 0){
  ROS_FATAL("Background_substraction::computeBackground: no Training data!");
  return false;
 }

 for (int x=0; x<dists[0].cols; ++x)
  for (int y=0; y<dists[0].rows; ++y){

   // iterate over all distimages and copy all valid dists
   vector<float> d;
   for (uint i=0; i<dists.size(); ++i){
    float c_d = dists[i].at<float>(y,x);
    if (c_d > 0)
     d.push_back(c_d);
   }

   uint valid_dists = d.size();

   if (valid_dists == 0){
    // means.at<float>(y,x)stays -1
    // vars.at<float>(y,x) stays -1
    continue;
   }


   if (valid_dists == 1){
    means.at<float>(y,x) = d[0];
    // vars.at<float>(y,x) stays -1
    continue;
   }

   assert(valid_dists >= 2);

   // compute mean
   float mu = 0;
   for (uint i=0; i<valid_dists; ++i)
    mu += d[i]/valid_dists;

   means.at<float>(y,x) = mu;

   // compute variance:
   float var = 0;
   for (uint i=0; i<valid_dists; ++i)
    var += pow(d[i]-mu,2)/valid_dists;

//   if (var > 0.3){
//    cout << var <<": ";
//    for (uint i=0; i<valid_dists; ++i)
//     cout << d[i] << " ";
//     cout << endl;
//   }


   std_dev.at<float>(y,x) = sqrt(var);

  }

// ROS_INFO("Background computed");


// double min_val, max_val;
//
// cv::minMaxLoc(means, &min_val, &max_val);
// ROS_INFO("means: %f %f", min_val, max_val);
// means -= min_val;
// means /= (max_val-min_val);
//
// cv::namedWindow("mu");
// cv::imshow("mu", means);



 mask = cv::Mat(dists[0].rows, dists[0].cols, CV_8UC1);
 mask.setTo(0);


 for (int x=0; x<dists[0].cols; ++x)
  for (int y=0; y<dists[0].rows; ++y){
   float std = std_dev.at<float>(y,x);

   mask.at<uchar>(y,x) = (std > max_std_dev || std < 0)?0:255;

  }

// cv::dilate(mask, mask, cv::Mat());
 cv::erode(mask, mask, cv::Mat());

//
// //vars -= min_val;
// // vars /= 0.02*0.02;
//
// // show points without any measurement (they have var = -100
// cv::Mat nans;
// std_dev *= -1;
// cv::threshold(std_dev, nans, 50, 1,CV_THRESH_BINARY);
// std_dev *= -1;
//
//
// cv::minMaxLoc(std_dev, &min_val, &max_val);
//
// ROS_INFO("var: %f %f", min_val, max_val);
//
//
// cv::minMaxLoc(std_dev, &min_val, &max_val);
//
//
//
//
//
//
//
// cv::Mat m1;
// cv::threshold(std_dev, m1, max_error*max_error, 1, CV_THRESH_BINARY);
//// cv::erode(vars, vars, cv::Mat());
//// cv::dilate(vars, vars, cv::Mat());
//
// ROS_INFO("var2: %f %f", min_val, max_val);
//
//// cv::Mat mask;
//// m1.copyTo(mask, nans);
//
// cv::namedWindow("var");
// cv::imshow("var", std_dev);
//
// cv::namedWindow("mask");
// cv::imshow("mask", mask);
//
// cv::namedWindow("nans");
// cv::imshow("nans", nans);


 cv::namedWindow("mask");
 cv::imshow("mask", mask);

 cv::waitKey(10);




 return true;


}
