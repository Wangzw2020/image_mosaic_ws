#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include "opencv2/imgproc/detail/distortion_model.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#define _NODE_NAME_ "image_mosaic"

using namespace cv;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

class ImageMosaic
{
private:
	ros::Publisher mosaic_image_pub_;
	std::string image_topic_;
	std::vector<int> image_id_;
	std::vector<std::string> image_topics_;
	message_filters::Subscriber<sensor_msgs::Image>* sub_image1_;
	message_filters::Subscriber<sensor_msgs::Image>* sub_image2_;
	message_filters::Subscriber<sensor_msgs::Image>* sub_image3_;
	message_filters::Subscriber<sensor_msgs::Image>* sub_image4_;
	message_filters::Synchronizer<SyncPolicy>* sync_;
	
public:
	ImageMosaic();
	bool init();
	void mosaicpub( const sensor_msgs::ImageConstPtr& msg1, 
					const sensor_msgs::ImageConstPtr& msg2,
					const sensor_msgs::ImageConstPtr& msg3, 
					const sensor_msgs::ImageConstPtr& msg4);
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	ImageMosaic mosaic;
	mosaic.init();
	ros::spin();
	return 0;
}

ImageMosaic::ImageMosaic()
{

}

bool ImageMosaic::init()
{
	ros::NodeHandle nh, nh_private("~");
	nh_private.param<std::string>("image_topic", image_topic_, "");
	if(!ros::param::get("~image_id",image_id_))
	{
		ROS_ERROR("[%s]: please set image id!",_NODE_NAME_);
		return false;
	}
	
	for(int i=0; i<image_id_.size(); ++i)
	{
		std::string topic = image_topic_ + std::to_string(image_id_[i]);
		image_topics_.push_back(topic);
	}
	
	mosaic_image_pub_ = nh.advertise<sensor_msgs::Image>("/image_mosaic", 1);
	sub_image1_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, image_topics_[0], 1);
	sub_image2_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, image_topics_[1], 1);
	sub_image3_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, image_topics_[2], 1);
	sub_image4_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, image_topics_[3], 1);;
	
	sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_image1_, *sub_image2_, *sub_image3_, *sub_image4_);
	sync_->registerCallback(boost::bind(&ImageMosaic::mosaicpub, this, _1, _2, _3, _4));
	
	ROS_INFO("[%s]: image_mosaic initial ok.!",_NODE_NAME_);
}

void ImageMosaic::mosaicpub(const sensor_msgs::ImageConstPtr& msg1, 
							const sensor_msgs::ImageConstPtr& msg2,
							const sensor_msgs::ImageConstPtr& msg3, 
							const sensor_msgs::ImageConstPtr& msg4)
{
	int mosaic_width = 0;
	int image_height, image_width;
	ROS_INFO("[%s]: image mosaic!",_NODE_NAME_);
	cv::Mat image1, image2, image3, image4;
	
	std::vector<cv_bridge::CvImagePtr> cv_ptr;
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8);
	image_width = cv->image.cols;
	image_height = cv->image.rows;
	mosaic_width += cv->image.cols;
	cv_ptr.push_back(cv);
	
	cv = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8);
	image_width = cv->image.cols;
	image_height = cv->image.rows;
	mosaic_width += cv->image.cols;
	cv_ptr.push_back(cv);

	cv = cv_bridge::toCvCopy(msg3, sensor_msgs::image_encodings::BGR8);
	image_width = cv->image.cols;
	image_height = cv->image.rows;
	mosaic_width += cv->image.cols;
	cv_ptr.push_back(cv);
	
	cv = cv_bridge::toCvCopy(msg4, sensor_msgs::image_encodings::BGR8);
	image_width = cv->image.cols;
	image_height = cv->image.rows;
	mosaic_width += cv->image.cols;
	cv_ptr.push_back(cv);

	cv::Mat mosaic_image(image_height, mosaic_width, CV_8UC3);
	mosaic_image.setTo(0);
	
	for (int i=0; i<image_id_.size(); ++i)
	{
		cv_ptr[i]->image.copyTo(mosaic_image(Rect(i * image_width, 0, image_width, image_height)));
	}
	
	sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mosaic_image).toImageMsg();
	imageMsg->header.frame_id = std::string("mosaic image");
	imageMsg->header.stamp = ros::Time::now();
	mosaic_image_pub_.publish(imageMsg);
	ROS_INFO("[%s]: mosaic image public!", _NODE_NAME_);
}
















