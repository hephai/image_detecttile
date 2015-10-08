/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, hephaistion GmbH
*  				 2014, JSK Lab.
*                2008, Willow Garage, Inc.
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

/********************************************************************
* image_detecttile_nodelet.cpp
* this is a forked version of image_rotate.
* this image_detecttile_nodelet supports:
*  1) nodelet
*  2) tf and tf2
*********************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <image_detecttile/ImageDetecttileConfig.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <opencv2/opencv.hpp>

#include <std_msgs/String.h>
#include <sstream>


#include <image_detecttile/image_detecttile_single.h>
#include <image_detecttile/image_detecttile_all.h>


namespace image_detecttile {
class ImageDetecttileNodelet : public nodelet::Nodelet
{

//  bool use_tf2_;
//  boost::shared_ptr<tf::TransformListener> tf_sub_;
//  tf::TransformBroadcaster tf_pub_;
//  boost::shared_ptr<tf2_ros::BufferClient> tf2_client_;
  image_detecttile::ImageDetecttileConfig config;
  dynamic_reconfigure::Server<image_detecttile::ImageDetecttileConfig> srv;

  image_transport::Publisher img_pub_;
  image_transport::Publisher img_pub2_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  double canny_low_thresh ;
  double canny_low_thresh2 ;
  double epsilonApproxPolyDP;
//  tf::Stamped<tf::Vector3> target_vector_;
//  tf::Stamped<tf::Vector3> source_vector_;
    
  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_;

  int subscriber_count_;
  ros::Time prev_stamp_;

  ros::Publisher chatter_pub;
  ros::Publisher image_detecttile_msg_pub;
  ::image_detecttile::image_detecttile_single single_msg;
  ::image_detecttile::image_detecttile_all all_msg;


//  // struktur zum sortieren
//  struct Areas {
//	  int areaNumber;
//	  double area;
//  };
//
//  // sort funktion
//  bool sortByArea(Areas &lhs, Areas &rhs) { return lhs.area > rhs.area; }



//  void setupTFListener()
//  {
//    if (use_tf2_) {
//      // shutdown tf_sub_
//      if (tf_sub_) {
//        tf_sub_.reset();
//      }
//    }
//    else {
//      if(!tf_sub_) {
//        tf_sub_.reset(new tf::TransformListener());
//      }
//    }
//  }
  
  void reconfigureCallback(image_detecttile::ImageDetecttileConfig &config, uint32_t level)
  {

	  if (canny_low_thresh != config.canny_low_thresh) {
		  canny_low_thresh = config.canny_low_thresh;
		  ROS_INFO("Reconfigure Request canny_low_thresh: %f", config.canny_low_thresh );
	  }
	  if (canny_low_thresh2 != config.canny_low_thresh2) {
		  canny_low_thresh2 = config.canny_low_thresh2;
		  ROS_INFO("Reconfigure Request canny_low_thresh2: %f", config.canny_low_thresh2 );
	  }
	  if (epsilonApproxPolyDP != config.epsilonApproxPolyDP) {
		  epsilonApproxPolyDP = config.epsilonApproxPolyDP;
		  ROS_INFO("Reconfigure Request epsilonApproxPolyDP: %f", config.epsilonApproxPolyDP );
	  }

//    target_vector_.setValue(config_.target_x, config_.target_y, config_.target_z);
//    source_vector_.setValue(config_.source_x, config_.source_y, config_.source_z);
//    if (subscriber_count_)
//    { // @todo Could do this without an interruption at some point.
//      unsubscribe();
//      subscribe();
//    }
//    if (use_tf2_ != config_.use_tf2) {
//      use_tf2_ = config_.use_tf2;
//      setupTFListener();
//    }
  }

  const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    do_work(msg, cam_info->header.frame_id);
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    do_work(msg, msg->header.frame_id);
  }

  void transformVector(const std::string& input_frame_id, const ros::Time& target_time,
                       const std::string& source_frame_id, const ros::Time& time,
                       const std::string& fixed_frame_id,
                       const tf::Stamped<tf::Vector3>& input_vector,
                       tf::Stamped<tf::Vector3>& target_vector,
                       const ros::Duration& duration)
  {
//    if (use_tf2_) {
//      geometry_msgs::TransformStamped trans
//        = tf2_client_->lookupTransform(input_frame_id, source_frame_id,
//                                       target_time, duration);
//      // geometry_msgs -> eigen -> tf
//      Eigen::Affine3d transform_eigen;
//      tf::transformMsgToEigen(trans.transform, transform_eigen);
//      tf::StampedTransform transform_tf; // convert trans to tfStampedTransform
//      tf::transformEigenToTF(transform_eigen, transform_tf);
//      tf::Vector3 origin = tf::Vector3(0, 0, 0);
//      tf::Vector3 end = input_vector;
//      tf::Vector3 output = (transform_tf * end) - (transform_tf * origin);
//      target_vector.setData(output);
//      target_vector.stamp_ = input_vector.stamp_;
//      target_vector.frame_id_ = input_frame_id;
//    }
//    else {
//      tf_sub_->waitForTransform(input_frame_id, target_time,
//                                source_frame_id, time,
//                                fixed_frame_id, duration);
//      tf_sub_->transformVector(input_frame_id, target_time, input_vector,
//                               fixed_frame_id, target_vector);
//    }
  }
  
  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
  {
//    try
    {
//      std::string input_frame_id = frameWithDefault(config_.input_frame_id, input_frame_from_msg);

//      // Transform the target vector into the image frame.
//      target_vector_.stamp_ = msg->header.stamp;
//      target_vector_.frame_id_ = frameWithDefault(config_.target_frame_id, input_frame_id);
//      tf::Stamped<tf::Vector3> target_vector_transformed;
//      transformVector(input_frame_id, msg->header.stamp,
//                      target_vector_.frame_id_, target_vector_.stamp_,
//                      input_frame_id, target_vector_, target_vector_transformed,
//                      ros::Duration(0.2));
//
//      // Transform the source vector into the image frame.
//      source_vector_.stamp_ = msg->header.stamp;
//      source_vector_.frame_id_ = frameWithDefault(config_.source_frame_id, input_frame_id);
//      tf::Stamped<tf::Vector3> source_vector_transformed;
//      transformVector(input_frame_id, msg->header.stamp,
//                      source_vector_.frame_id_, source_vector_.stamp_,
//                      input_frame_id, source_vector_, source_vector_transformed,
//                      ros::Duration(0.01));

      // NODELET_INFO("target: %f %f %f", target_vector_.x(), target_vector_.y(), target_vector_.z());
      // NODELET_INFO("target_transformed: %f %f %f", target_vector_transformed.x(), target_vector_transformed.y(), target_vector_transformed.z());
      // NODELET_INFO("source: %f %f %f", source_vector_.x(), source_vector_.y(), source_vector_.z());
      // NODELET_INFO("source_transformed: %f %f %f", source_vector_transformed.x(), source_vector_transformed.y(), source_vector_transformed.z());

    }
//    catch (tf::TransformException &e)
//    {
//      NODELET_ERROR("Transform error: %s", e.what());
//    }

    // Publish the transform.
//    tf::StampedTransform transform;
//    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//    transform.setRotation(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), angle_));
//    transform.frame_id_ = msg->header.frame_id;
//    transform.child_frame_id_ = frameWithDefault(config_.output_frame_id, msg->header.frame_id + "_rotated");
//    transform.stamp_ = msg->header.stamp;
//    tf_pub_.sendTransform(transform);

    // Transform the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

      cv::Mat canny_output;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::Canny(in_image, canny_output, canny_low_thresh, canny_low_thresh*2, 3);
      cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
      //  CV_RETR_TREE, CV_RETR_EXTERNAL

      /// Find the hull object for each contour
      std::vector<std::vector<cv::Point> >hull( contours.size() );

      for( int i = 0; i < contours.size(); i++ )
         {
    	    // die convexHull liefert mehr als 4 points für die perspTrans, müsste man wieder aussortieren
    	    //convexHull( cv::Mat(contours[i]), hull[i], false );
    	  	// die approxPolyDP liefert immer 4 points
         	cv::approxPolyDP(cv::Mat(contours[i]), hull[i], epsilonApproxPolyDP, true);
         }

      cv::Mat out_image = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
      int colordelta = std::abs(255/contours.size());
      int maxAreaHull = 0;
      int maxAreaHullI = 1;
      for( int i = 0; i< contours.size(); i++ )
         {
           cv::Scalar color = cv::Scalar( 255-(i*colordelta), 0, (i*colordelta) ); //von rot nach blau
           //von rot nach blau
           cv::drawContours( out_image, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
           // hull in weiss
           cv::drawContours( out_image, hull, i, cv::Scalar( 255, 255, 255 ), 1, 8, hierarchy, 0, cv::Point() );
           // die hull suchen mit der grössten area
           if (cv::contourArea(hull[i], false) > maxAreaHull) {
        	   maxAreaHull = cv::contourArea(hull[i], false);
        	   maxAreaHullI = i;
           }
         }
      // die grösste area in giftgrün einzeichnen
      cv::drawContours( out_image, hull, maxAreaHullI, cv::Scalar( 0, 255, 0 ), 2, 8, hierarchy, 0, cv::Point() );

      // von der hull das min area rect ausrechnen, um genau 4 Punkte für die perspTrans zu bekommen
//      cv::RotatedRect srcRefRect = cv::minAreaRect(hull[maxAreaHull]);


	  // bei der gleichen Auflösung bleiben 640*480? nein
      // DinA4 = 210*297 mm, damit sollten die pixel quadratisch sein und 0.1mm entsprechen
	  cv::Mat out_image2 = cv::Mat::zeros( 2100, 2970, CV_8UC3 );


      // src points aus der hull[maxAreaHullI] rausziehen, geht nicht mit convexHull, mehr als 4 points
      // funktioniert aber mit approxPolyDP, wenn epsilon gross genug, dann 4 Punkte
      if (hull[maxAreaHullI].size()==4) {
		  cv::Point2f srcQuad[] = {
			hull[maxAreaHullI] [0],
			hull[maxAreaHullI] [1],
			hull[maxAreaHullI] [2],
			hull[maxAreaHullI] [3]
		  };

		  // dst points sind das Din A4 Blatt 210*297 mm
		  // offenbar kommen die hull Punkte von links unten clockwise, lu lo, ro, ru

		  float factor = 1;
		  cv::Point2f dstQuad[] =
		  {
			cv::Point2f(0.0f, 0.0f),
			cv::Point2f(0.0f, factor*2100.0f),
			cv::Point2f(factor*2970.0f, factor*2100.0f),
			cv::Point2f(factor*2970.0f,  0.0f)
		  };


		//      cv::Mat perspective_matrix = cv::getPerspectiveTransform(hull[maxAreaHullI], dstQuad);
		  cv::Mat perspectiveMat = cv::getPerspectiveTransform(srcQuad, dstQuad);

		  // um die tiles einzuzeichnen, muss ich nur die contours nehmen, die in der hierarchy direkt unter hull[maxAreaHullI] liegen
		  // wie warpPerspective ich die hulls?

		  // die hulls, die in der hierarchy direkt unterhalb von maxAreaHullI, maxLevel = 1 in giftgrün einzeichnen
		  // geht nicht, ich muss die hull finden die mehr als ein Kind hat, das sind dann die äussersten hulls der tiles
//		  ROS_INFO("list of hierarchy under maxAreaHullI %i: ;0: %i  ;1: %i; 2: %i  ;3: %i", maxAreaHullI, hierarchy[maxAreaHullI] [0], hierarchy[maxAreaHullI] [1], hierarchy[maxAreaHullI] [2], hierarchy[maxAreaHullI] [3] );
//		  ROS_INFO("list of hierarchy under hierarchy[maxAreaHullI] [2] %i: ;0: %i  ;1: %i; 2: %i  ;3: %i", hierarchy[maxAreaHullI] [2], hierarchy[hierarchy[maxAreaHullI] [2]] [0], hierarchy[hierarchy[maxAreaHullI] [2]] [1], hierarchy[hierarchy[maxAreaHullI] [2]] [2], hierarchy[hierarchy[maxAreaHullI] [2]] [3] );
//		  ROS_INFO("list of hierarchy under hierarchy[hierarchy[maxAreaHullI] [2]] [2] %i: ;0: %i  ;1: %i; 2: %i  ;3: %i", hierarchy[hierarchy[maxAreaHullI] [2]] [2], hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [0], hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [1], hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [2], hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [3] );
//		  ROS_INFO("list of hierarchy under hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [2] %i: ;0: %i  ;1: %i; 2: %i  ;3: %i", hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [2], hierarchy[hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [2]] [0], hierarchy[hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [2]] [1], hierarchy[hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [2]] [2], hierarchy[hierarchy[hierarchy[hierarchy[maxAreaHullI] [2]] [2]] [2]] [3] );

		  cv::drawContours( in_image, hull, maxAreaHullI, cv::Scalar( 0, 255, 0 ), 1, 8, hierarchy, 0, cv::Point() );

		  // hierarchy muss so aussehen: hierarchy[maxAreaHullI] (contour DinA4 Blatt) hat child hole ->  hierarchy[maxAreaHullI] [2]
		  // im hole liegen die contours der tiles als child mit dem hole als parent alles  wo hierarchy[i] [3] == hierarchy[maxAreaHullI] [2]


		  std::vector<std::vector<cv::Point2f> >hullTiles;

		  for ( int i = 0; i< hierarchy.size(); i++ ) {
			  if (hierarchy[i][3] == hierarchy[maxAreaHullI] [2]) {
//				  cv::drawContours( in_image, hull, i, cv::Scalar( 0, 255, 0 ), 1, 8, hierarchy, 0, cv::Point() );

				  // um die cv::Points aus hull in cv::Points2f umzucasten (für perspectiveTransform)
				  std::vector<cv::Point2f> tilePoints2f;
				  for ( int j = 0; j< hull[i].size(); j++) {
					  tilePoints2f.push_back(hull[i] [j]);
				  }
				  hullTiles.push_back(tilePoints2f);
//				  ROS_INFO("hierarchy_id: %i; tilePoints2f.size: %i",i, tilePoints2f.size() );
			  }
		  }
		  ROS_INFO("%i Tiles gefunden", hullTiles.size());

		  cv::warpPerspective(in_image, out_image2, perspectiveMat, out_image2.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

		  // jetzt kann ich mit minAreaRect anfangen, vor dem warp sind es keine Quadrate, Rechtecke, ...
		  // dazu die hullTiles warpen
		  std::vector<std::vector<cv::Point2f> >hullTilesWarped(hullTiles.size());
		  std::vector<cv::RotatedRect>rectTiles(hullTiles.size());
		  std::vector<cv::Vec3b> rgbColorCenter(hullTiles.size());
		  for ( int i = 0; i < hullTiles.size(); i++) {
			  cv::perspectiveTransform(hullTiles[i],hullTilesWarped[i],perspectiveMat);

			  // und gleich das minAreaRect ausrechnen
			  rectTiles[i] = cv::minAreaRect(hullTilesWarped[i]);
			  cv::circle(out_image2, rectTiles[i].center,4,cv::Scalar(0,255,0),3,8,0);
			  // beschriften
//			  std::string strIndex;
//			  strIndex.push_back((char) i);
			  char charIndex [3];
			  std::sprintf(charIndex, "%i",i);
			  cv::putText(out_image2, charIndex,rectTiles[i].center,cv::FONT_HERSHEY_PLAIN, 3.0,cv::Scalar(0,255,0),2 , 8, false);

			  // die Farbe des Tiles rausbekommen,
			  rgbColorCenter[i] = out_image2.at<cv::Vec3b>(rectTiles[i].center);


			  // kästchen um die tiles malen
			  cv::Point2f polyPoints[4];
			  rectTiles[i].points(polyPoints);
			  for (int j = 0; j < 4; j++ ) {
				  cv::line(out_image2, polyPoints[j], polyPoints[(i+1)%4], cv::Scalar(0,255,255), 2);
			  }
		  }




		  // hier kommt der Publisher, der Informationen zu jedem Tile raushaut
//		  std_msgs::String msg;
//		  std::stringstream ss;
//		  ss << " ", rectTiles.size(), " tiles \n ";

		  all_msg.anzTiles = rectTiles.size();
		  for ( int i = 0; i < rectTiles.size(); i++) {
//			  ss << " tile_idx: " << i << "; centerPoint.x: " << rectTiles[i].center.x << "; centerPoint.y: " << rectTiles[i].center.y << "; size.height: " << rectTiles[i].size.height << "; size.width: " << rectTiles[i].size.width << "; angle: " << rectTiles[i].angle << "; RGB color" << rgbColorCenter[i] << " \n" ;

			  single_msg.tile_idx = i;
			  single_msg.centerPoint_x = rectTiles[i].center.x;
			  single_msg.centerPoint_y = rectTiles[i].center.y;
			  single_msg.size_height = rectTiles[i].size.height;
			  single_msg.size_width = rectTiles[i].size.width;
			  single_msg.angle = rectTiles[i].angle;
			  single_msg.color_B = rgbColorCenter[i].val[1]; // RGB wird angeblich als BGR gespeichert
			  single_msg.color_G = rgbColorCenter[i].val[2];
			  single_msg.color_R = rgbColorCenter[i].val[3];

			  all_msg.image_detecttile_vector.push_back(single_msg);

		  }
//		  msg.data = ss.str();
//		  chatter_pub.publish(msg);
		  image_detecttile_msg_pub.publish(all_msg);



		  // um die tiles einzuzeichnen, feststellen welche hull innerhalb der hull[maxAreaHullI] liegt
//		  bool isInHullMax = true;
//		  for(int i = 0; i< hull.size(); i++)
//		  {
//			  //for(int j = 0; j<hull[i].size() && isInHullMax; j++)
//			  int j = 0;
//			  while (cv::pointPolygonTest(hull[maxAreaHullI], hull[i][j], false) && j<hull[i].size())
//			  {
//				  j+=1;
//			  }
//			  if (j == hull[i].size())
//			  {
//			  	cv::drawContours( out_image2, hull, i, cv::Scalar( 0, 255, 0 ), 2, 8, hierarchy, 0, cv::Point() );
//			  }
//		  }

//		  // um die tiles einzuzeichnen, hier das gleiche wie vorher jetzt aber mit out_image2
//	      cv::Mat canny_output2;
//	      std::vector<std::vector<cv::Point> > contours2;
//	      std::vector<cv::Vec4i> hierarchy2;
//
//	      cv::Canny(out_image2, canny_output2, canny_low_thresh2, canny_low_thresh2*2, 3);
//	      cv::findContours(canny_output2, contours2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
//
//	      /// Find the hull object for each contour
//	      std::vector<std::vector<cv::Point> >hull2( contours2.size() );
//
//	      for( int i = 0; i < contours2.size(); i++ )
//	         {
//	    	    // die convexHull liefert mehr als 4 points für die perspTrans, müsste man wieder aussortieren
//	    	    convexHull( cv::Mat(contours2[i]), hull2[i], true );
//	    	  	// die approxPolyDP
//	         	//cv::approxPolyDP(cv::Mat(contours[i]), hull[i], epsilonApproxPolyDP, true);
//	    	    //alle contours in giftgrün malen
//	    	    cv::drawContours( out_image2, hull2, i, cv::Scalar( 0, 255, 0 ), 2, 8, hierarchy2, 0, cv::Point() );
//	         }





      }



//      // die contour suchen mit 4 Ecken und der grössten area
//
//      std::vector<Areas> hullAreas(hull.size());
//
//      for (int i=0; i < hull.size(); i++) // nur die contours, die in der hierarchy mit [0] auftauchen
//      {
//    	  hullAreas[i].areaNumber = i;
//    	  hullAreas[i].area = cv::contourArea(hull[i]); // + contours[contour_number].size()/2 + 1;

// die holes will ich nicht abziehen
//    	  for (int hole_number = hierarchy[contour_number][2]; (hole_number>=0); hole_number=hierarchy[hole_number][0])
//    		  area -= ( contourArea(contours[hole_number]) – contours[hole_number].size()/2 + 1 );
//      }

//      std::sort(hullAreas.begin(), hullAreas.end(), std::greater<Areas>() );
//      std::sort(hullAreas.begin(),hullAreas.end(), ImageDetecttileNodelet::sortByArea);
//      std::sort(hullAreas.begin(),hullAreas.end());



      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
//      out_img->header.frame_id = transform.child_frame_id_;
      img_pub_.publish(out_img);

      // Publish the image2.
      sensor_msgs::Image::Ptr out_img2 = cv_bridge::CvImage(msg->header, msg->encoding, out_image2).toImageMsg();
//      out_img->header.frame_id = transform.child_frame_id_;
      img_pub2_.publish(out_img2);

    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
//    if (config_.use_camera_info && config_.input_frame_id.empty())
//      cam_sub_ = it_->subscribeCamera("image", 3, &ImageRotateNodelet::imageCallbackWithInfo, this);
//    else
      img_sub_ = it_->subscribe("image", 3, &ImageDetecttileNodelet::imageCallback, this);
  }

  void unsubscribe()
  {
      NODELET_DEBUG("Unsubscribing from image topic.");
      img_sub_.shutdown();
      cam_sub_.shutdown();
  }

  void connectCb(const image_transport::SingleSubscriberPublisher& ssp)
  {
    if (subscriber_count_++ == 0) {
      subscribe();
    }
  }

  void disconnectCb(const image_transport::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
    if (subscriber_count_ == 0) {
      unsubscribe();
    }
  }

public:
  virtual void onInit()
  {
    nh_ = getNodeHandle();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
//    tf2_client_.reset(new tf2_ros::BufferClient("tf2_buffer_server", 100, ros::Duration(0.2)));
    subscriber_count_ = 0;
    prev_stamp_ = ros::Time(0, 0);
    image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&ImageDetecttileNodelet::connectCb, this, _1);
    image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&ImageDetecttileNodelet::disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "contour")).advertise("image", 1, connect_cb, disconnect_cb);
    img_pub2_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "tilefield")).advertise("image", 1, connect_cb, disconnect_cb);

    chatter_pub = nh_.advertise<std_msgs::String>("centerpoints",1000);
    image_detecttile_msg_pub = nh_.advertise<image_detecttile::image_detecttile_all>("image_detecttile_msg",1000);

    dynamic_reconfigure::Server<image_detecttile::ImageDetecttileConfig>::CallbackType f =
      boost::bind(&ImageDetecttileNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);
  }
};
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_detecttile::ImageDetecttileNodelet, nodelet::Nodelet);
