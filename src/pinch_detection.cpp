/*
 * pinch_detection.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#include "pinch_recognition/pinch_detection.h"

using namespace std;




/**
* Aka: Is there a hand visible?
*
* @param contour_infos
* @return
*/
/// returns true if at least on contour crosses the border of the sandbox
bool atLeastOneCrossing(const vector<Contour_info> & contour_infos){
 for (uint i=0; i<contour_infos.size(); ++i)
  if (contour_infos[i].border_crossing)
   return true;
 return false;
}



float getAreaOfTriangle(const pcl_Point& a, const pcl_Point& b, const pcl_Point& c){

 if (a.x!=a.x) return 0;
 if (b.x!=b.x) return 0;
 if (c.x!=c.x) return 0;

 pcl_Point u = sub(b,a);
 pcl_Point v = sub(c,a);


// ROS_INFO("A: %f %f %f", a.x,a.y,a.z);
// ROS_INFO("B: %f %f %f", b.x,b.y,b.z);
// ROS_INFO("C: %f %f %f", c.x,c.y,c.z);


 // compute cross product:
 pcl_Point cr;
 cr.x = u.y*v.z-u.z*v.y;
 cr.y = u.z*v.x-u.x*v.z;
 cr.z = u.x*v.y-u.y*v.x;

 // ROS_INFO("Are/a: %f", norm(cr)/2);

 return norm(cr)/2;
}

/**
*
* @param object
* @param cloud
* @return
*/
/// get size of object in cm^2
float getArea(const cv::Mat& object, const Cloud& cloud){

 assert(object.type() == CV_8UC1);

 // check every triangle in the image and

 // todo (?) erode object to get rid of measurements at the edge of objects
 // errors could otherwise dominate the area of the object

 float total_area = 0;

 // code not perfect: if only top right corner is nan, both triangles will have no area
 // TODO:
 for (int x=0; x<object.cols-1; ++x)
  for (int y=0; y<object.rows-1; ++y){

   // check if the pixels on the corner of the rectangle with (x,y) as top left corner are on the object
   bool tl = object.at<uchar>(y,x) > 0;
   bool tr = object.at<uchar>(y,x+1) > 0;
   bool ll = object.at<uchar>(y+1,x) > 0;
   bool lr = object.at<uchar>(y+1,x+1) > 0;

   pcl_Point tl_p = cloud.at(x,y);
   pcl_Point tr_p = cloud.at(x+1,y);
   pcl_Point ll_p = cloud.at(x,y+1);
   pcl_Point lr_p = cloud.at(x+1,y+1);

   // check upper left triangle:
   if (tl && tr && ll){
    total_area += getAreaOfTriangle(tl_p, tr_p, ll_p); // returns 0 if one of the points contains NaN
   }

   // check lower left triangle:
   if (tr && ll && lr){
    total_area += getAreaOfTriangle(tr_p, ll_p, lr_p); // returns 0 if one of the points contains NaN
   }

  }

 return total_area;

}


void detectionHelper(cv::Mat& foreground, const cv::Mat& areaMask, const Cloud& cloud, vector<vector<cv::Point> >& contours, vector<cv::Vec4i>& hierarchy, vector<Contour_info> & contour_infos){

 contours.clear();
 hierarchy.clear();
 contour_infos.clear();

 cv::findContours(foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 // create image with edges of sandbox-area
 cv::Mat edge;
 cv::Canny(areaMask, edge, 1,10);
 cv::dilate(edge, edge, cv::Mat(), cv::Point(-1,-1),1);

 cv::Mat contour_image(foreground.size(), CV_8UC1);
 cv::Mat intersection(foreground.size(), CV_8UC1);
 Contour_info info;

 cv::Scalar color(255,255,255);

 for( uint i = 0; i< contours.size(); i++ ){
  info.area = cv::contourArea(contours[i]);

  contour_image.setTo(0);
  intersection.setTo(0);

  // draw contour on image
  cv::drawContours( contour_image, contours, i, color, -1, 8, hierarchy, 0, cv::Point() );


 // float area_m2 = getArea(contour_image, cloud);
 // ROS_INFO("contour %i: pix: %f, cm^2: %f", i,info.area,area_m2*100*100);

  // compute intersection between edges of sandbox and contour
  contour_image.copyTo(intersection, edge);

  // if there are any points in the intersection, the contour is probably a hand
  info.border_crossing = cv::countNonZero(intersection)>0;

  cv::Moments mom = cv::moments(contours[i]);
  info.center = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);

  contour_infos.push_back(info);
 }

 assert(contour_infos.size() == contours.size());

}

/**
*
*
*
* @param foreground
* @param areaMask
* @param scene
* @param objects
* @param border_crossing
* @param col
*
* @todo (vll) compute area of objects in cm and not only in pixels
*/
void detectPlayingPieces2(cv::Mat& foreground, const cv::Mat& areaMask, const Cloud& scene, std::vector<Playing_Piece>& objects, bool& border_crossing,  cv::Mat* col){

 assert(foreground.cols == int(scene.width));
 objects.clear();

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;
 vector<Contour_info> contour_infos;

 // detectionHelper(foreground, areaMask, contours, hierarchy,contour_infos);

 border_crossing = atLeastOneCrossing(contour_infos);

 // create objects:
 for (uint i=0; i<contour_infos.size(); ++i){

  float area = contour_infos[i].area;

  if (area < C_MIN_OBJECT_AREA)
   continue;

  Playing_Piece pp; // (contour_infos[i].center);
  pp.detection_center = contour_infos[i].center;
  pp.area = contour_infos[i].area;
  objects.push_back(pp);
 }


}




void detectPlayingPieces(cv::Mat& foreground, const cv::Mat& areaMask, const Cloud& scene, std::vector<Playing_Piece>& objects, bool& large_component_visible, bool& border_crossing,  cv::Mat* col){

 assert(foreground.cols == int(scene.width));
 objects.clear();

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 // < 0.5 ms
 // ros::Time start = ros::Time::now();
 cv::findContours(foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
 // ROS_INFO("findContours: %f ms", (ros::Time::now()-start).toSec()*1000.0);


 cv::Mat edge;
 // get boundary of modeled are within the image:
 cv::Canny(areaMask, edge, 1,10);

 // cv::imwrite("canny.png", edge);

 cv::dilate(edge, edge, cv::Mat(), cv::Point(-1,-1),1);

 // cv::imwrite("dilate.png", edge);


 large_component_visible = false;

 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);

  if (areas[i] > C_MIN_HAND_AREA)
   large_component_visible = true;

 }

 cv::Scalar color =  cv::Scalar( 255,0,0 );


 cv::Mat contour_image(foreground.size(), CV_8UC1);
 contour_image.setTo(0);

 for( uint i = 0; i< contours.size(); i++ ){

  ROS_INFO("contour %i has size %f",i,areas[i]);


  if (areas[i] < C_MIN_OBJECT_AREA)
   continue;



  cv::Moments mom = cv::moments(contours[i]);

  cv::Point2f center = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);

  Playing_Piece pp;
  pp.detection_center = center;
  pp.area = areas[i];
  objects.push_back(pp);

  cv::drawContours( contour_image, contours, i, color, -1, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
 }

 // cv::imwrite("contours.png", contour_image);

 cv::Mat intersection; intersection.setTo(0);
 contour_image.copyTo(intersection, edge);

 border_crossing = cv::countNonZero(intersection)>0;

 if (border_crossing){
  ROS_INFO("FOUND INTERSECTION WITH BORDER");
 }

}


/**
*
* @param foreground
* @param areaMask
* @param grasps
* @param objects
*/
/// Combined computation of Objects on the surface and hand signs
void detectGraspAndObjects(cv::Mat& foreground, const cv::Mat& areaMask, const Cloud& cloud, bool & border_crossing, std::vector<Grasp>* grasps, std::vector<Playing_Piece>* objects, cv::Mat* col){

 // assert(grasp || objects);

 if (grasps) grasps->clear();
 if (objects) objects->clear();


 if (!(grasps || objects)){
  ROS_WARN("detectGrasp: at least one of grasp or objects should be given...");
  return;
 }

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;
 vector<Contour_info> contour_infos;

 detectionHelper(foreground, areaMask, cloud, contours, hierarchy,contour_infos);

 // border_crossing = atLeastOneCrossing(contour_infos);

 border_crossing = false;

 for (int i=0; i < int(contour_infos.size()); ++i){

  if (objects){
   // add new object detection if
   Playing_Piece pp;
   pp.detection_center = contour_infos[i].center;
   pp.area = contour_infos[i].area;
   if (pp.area >= C_MIN_OBJECT_AREA){
    objects->push_back(pp);

    if (contour_infos[i].border_crossing)
     border_crossing = true;

    // every object is shown in red (can be recolored if used in grasp-detection
    if (col)
     cv::drawContours( *col, contours, i, CV_RGB(255,0,0), 2, 8, hierarchy, 0, cv::Point() );
   }
  }

  if (!grasps || !border_crossing)
   continue;

  // grasp detection:
  bool is_root = hierarchy[i][3] < 0;
  bool border_crossing = contour_infos[i].border_crossing;

  // Hand has no parent contour and crosses the border of the sandbox
  if (!(is_root && border_crossing))
   continue;

  // find largest hole within the component:
  int largest_child = -1;
  float area_of_largest_child = 100; // minimal area of hole in component // TODO: compute area in cm, not in pixels

  for (uint j = 0; j<contour_infos.size(); ++j){
   int parent = hierarchy[j][3];

   if (parent == i && contour_infos[j].area > area_of_largest_child){
    area_of_largest_child = area_of_largest_child;
    largest_child = j;
   }
  }

  // if a hole with sufficient size was found, create new Grasp detection
  if (largest_child > 0){

   grasps->push_back(Grasp(contour_infos[largest_child].center));


   // blue parent contour
   // green hole
   if (col){
    cv::drawContours( *col, contours, i, CV_RGB(0,0,255), 2, 8, hierarchy, 0, cv::Point() );
    cv::drawContours( *col, contours, largest_child, CV_RGB(0,255,0), 2, 8, hierarchy, 0, cv::Point() );
   }

  }

 } // loop over all contours



}


void detectGrasp(cv::Mat& foreground, std::vector<Grasp>& grasps, cv::Mat* col, bool verbose){

 if (foreground.cols == 0){
  ROS_ERROR("no foreground detected!");
  return;
 }

 grasps.clear();

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours( foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 // ROS_INFO("found %zu contours", contours.size());

 // compute area for each contour:
 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);
 }


 cv::Mat drawing;
 if (verbose) drawing = cv::Mat::zeros( foreground.size(), CV_8UC3 );

 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = areas[i];
  //  ROS_INFO("%i %f", i, area);

  float large_area_threshold = 1500;
  float small_area_threshold = 100;

  bool has_parent = hierarchy[i][3] >= 0;
  bool is_small = area < small_area_threshold;
  bool is_large = area > large_area_threshold;
  bool parent_is_large = has_parent && areas[hierarchy[i][3]] > large_area_threshold;

  //  if (has_parent)
  //   ROS_INFO("Grasp: large: %f, small: %f", areas[hierarchy[i][3]], area);

  if (has_parent && is_small) continue;
  if (!has_parent && !is_large) continue;

  cv::Scalar color;

  if (area > large_area_threshold){
   color =  cv::Scalar( 255,0,0 );
  }
  else{
   color =  cv::Scalar( 0,0,255 );

   if (has_parent && !is_small && parent_is_large){
    color =  cv::Scalar( 0,255,0 );



    cv::Moments mom = cv::moments(contours[i]);

    cv::Point2f center = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);
    Grasp grasp(center);
    grasps.push_back(grasp);

    if (col)
     cv::circle(*col, center, 30, CV_RGB(255,0,0),-1);

   }

  }

  if (verbose)
   cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

 if (verbose){
  cv::namedWindow("contours");
  cv::imshow("contours", drawing);
  cv::waitKey(10);
 }


}





void Pinch_detector::applyMaskToForeground(const cv::Mat& mask){

 if(!(mask.cols == foreground.cols)){
  ROS_INFO("Mask: %i, foreground: %i",mask.cols,foreground.cols );
  assert(1==0);
 }
 cv::Mat masked;
 foreground.copyTo(masked, mask);
 foreground = masked;
}



void Grasp_detector::detectGrasps(std::vector<cv::Point2f>& res, float max_dist, cv::Mat* current, cv::Mat* col, bool verbose){


 res.clear();


 cv::Mat foreground = getForeground(max_dist, *current);

 // cv::namedWindow("foreorund");
 // cv::imshow("foreorund", foreground);
 // cv::waitKey(10);


 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours(foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 // compute area for each contour:
 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);
 }


 cv::Mat drawing;

 if (verbose) drawing = cv::Mat::zeros( foreground.size(), CV_8UC3 );

 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = areas[i];

  // ROS_INFO("%i %f", i, area);

  float large_area_threshold = 1500;
  float small_area_threshold = 100;

  bool has_parent = hierarchy[i][3] >= 0;
  bool is_small = area < small_area_threshold;
  bool is_large = area > large_area_threshold;
  bool parent_is_large = has_parent && areas[hierarchy[i][3]] > large_area_threshold;

  //  if (has_parent)
  //   ROS_INFO("Grasp: large: %f, small: %f", areas[hierarchy[i][3]], area);

  if (has_parent && is_small) continue;
  if (!has_parent && !is_large) continue;

  cv::Scalar color;

  if (area > large_area_threshold){
   color =  cv::Scalar( 255,0,0 );
  }
  else{
   color =  cv::Scalar( 0,0,255 );

   if (has_parent && !is_small && parent_is_large){
    color =  cv::Scalar( 0,255,0 );

    cv::Moments mom = cv::moments(contours[i]);
    res.push_back(cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00));
   }

  }
  if (verbose)
   cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

 if (verbose){
  cv::namedWindow("contours");
  cv::imshow("contours", drawing);
  cv::waitKey(10);
 }

}



bool Pinch_detector::detectGrasp(std::vector<cv::Point2f>& res, cv::Mat* col, bool verbose){

 if (foreground.cols == 0){
  ROS_ERROR("no foreground detected!");
  return false;
 }
 res.clear();

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours( foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 // compute area for each contour:
 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);
 }


 cv::Mat drawing;

 if (verbose) drawing = cv::Mat::zeros( foreground.size(), CV_8UC3 );

 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = areas[i];

  // ROS_INFO("%i %f", i, area);

  float large_area_threshold = 1500;
  float small_area_threshold = 100;

  bool has_parent = hierarchy[i][3] >= 0;
  bool is_small = area < small_area_threshold;
  bool is_large = area > large_area_threshold;
  bool parent_is_large = has_parent && areas[hierarchy[i][3]] > large_area_threshold;

  //  if (has_parent)
  //   ROS_INFO("Grasp: large: %f, small: %f", areas[hierarchy[i][3]], area);

  if (has_parent && is_small) continue;
  if (!has_parent && !is_large) continue;

  cv::Scalar color;

  if (area > large_area_threshold){
   color =  cv::Scalar( 255,0,0 );
  }
  else{
   color =  cv::Scalar( 0,0,255 );

   if (has_parent && !is_small && parent_is_large){
    color =  cv::Scalar( 0,255,0 );

    cv::Moments mom = cv::moments(contours[i]);
    res.push_back(cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00));
   }

  }
  if (verbose)
   cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

 if (verbose){
  cv::namedWindow("contours");
  cv::imshow("contours", drawing);
  cv::waitKey(10);
 }

 return res.size()> 0;

}




Cloud Pinch_detector::removeBackground(const Cloud& current,  float min_dist, float max_dist){

 // foreground is now a member variable
 Cloud result = Background_substraction::removeBackground(current, min_dist, max_dist, foreground);




 return result;

}
