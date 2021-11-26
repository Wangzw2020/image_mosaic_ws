#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include "opencv2/imgproc/detail/distortion_model.hpp"

#define _NODE_NAME_ "image_mosaic"

using namespace cv;

class ImageMosaic
{
private:
	ros::Publisher mosaic_image_pub_;
	std::vector<image_transport::Subscriber> image_sub_;
	std::string image_topic_;
	std::vector<int> image_id_;
	std::vector<std::string> image_topics_;
	std::vector<cv_bridge::CvImagePtr> cv_ptr_;
	cv::Size img_size_, new_img_size_;
	std::vector<int> imageResolution_;
	int frame_rate_;
	bool is_show_result_;
	std::vector<bool> image_ok_;
	bool image_all_ok_ = true;
	ros::Subscriber sub_image1_, sub_image2_, sub_image3_, sub_image4_;
	ros::Timer timer_;
	
public:
	ImageMosaic();
	bool init();
	void loadimage1(const sensor_msgs::ImageConstPtr& msg);
	void loadimage2(const sensor_msgs::ImageConstPtr& msg);
	void loadimage3(const sensor_msgs::ImageConstPtr& msg);
	void loadimage4(const sensor_msgs::ImageConstPtr& msg);
	void mosaicpub(const ros::TimerEvent&);
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
	nh_private.param<int>("frame_rate",frame_rate_,30);
	nh_private.param<bool>("is_show_result",is_show_result_,false);
	if(!ros::param::get("~image_id",image_id_))
	{
		ROS_ERROR("[%s]: please set image id!",_NODE_NAME_);
		return false;
	}
	
	for(int i=0; i<image_id_.size(); ++i)
	{
		bool image_ok = false;
		image_ok_.push_back(image_ok);
		std::string topic = image_topic_ + std::to_string(image_id_[i]);
		image_topics_.push_back(topic);
		cv_bridge::CvImagePtr cv;
		cv_ptr_.push_back(cv);
	}
	
	//image_transport::ImageTransport it(nh);
	mosaic_image_pub_ = nh.advertise<sensor_msgs::Image>("/image_mosaic", 1);
	
	sub_image1_ = nh.subscribe(image_topics_[0], 1, &ImageMosaic::loadimage1, this);
	sub_image2_ = nh.subscribe(image_topics_[1], 1, &ImageMosaic::loadimage2, this);
	sub_image3_ = nh.subscribe(image_topics_[2], 1, &ImageMosaic::loadimage3, this);
	sub_image4_ = nh.subscribe(image_topics_[3], 1, &ImageMosaic::loadimage4, this);
	
	timer_ = nh.createTimer(ros::Duration(0.1), &ImageMosaic::mosaicpub, this);
	
	ROS_INFO("image_mosaic initial ok.");
}

void ImageMosaic::loadimage1(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_ERROR("[%s]: getting image! %s",_NODE_NAME_, msg->header.frame_id.c_str());
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_ptr_[0] = cv;
	image_ok_[0] = true;
}

void ImageMosaic::loadimage2(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_ERROR("[%s]: getting image! %s",_NODE_NAME_, msg->header.frame_id.c_str());
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_ptr_[1] = cv;
	image_ok_[1] = true;
}

void ImageMosaic::loadimage3(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_ERROR("[%s]: getting image! %s",_NODE_NAME_, msg->header.frame_id.c_str());
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_ptr_[2] = cv;
	image_ok_[2] = true;
}

void ImageMosaic::loadimage4(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_ERROR("[%s]: getting image! %s",_NODE_NAME_, msg->header.frame_id.c_str());
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_ptr_[3] = cv;
	image_ok_[3] = true;
}

void ImageMosaic::mosaicpub(const ros::TimerEvent&)
{
	int width = 0;
	
	ROS_INFO("[%s]: image mosaic!",_NODE_NAME_);
	for (int i=0; i<image_id_.size(); ++i)
		if (image_ok_[i] == false)
			image_all_ok_ = false;
			
	if (image_all_ok_)
	{
		for (int i=0; i<image_id_.size(); ++i)
			width += cv_ptr_[i]->image.cols;
		int img_width = cv_ptr_[0]->image.cols;
		int height = cv_ptr_[0]->image.rows;
		cv::Mat mosaic_image(height, width, CV_8UC3);
		mosaic_image.setTo(0);
		
		for (int i=0; i<image_id_.size(); ++i)
			cv_ptr_[i]->image.copyTo(mosaic_image(Rect(i * img_width, 0, img_width, height)));
		sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mosaic_image).toImageMsg();
		imageMsg->header.frame_id = std::string("mosaic image");
		imageMsg->header.stamp = ros::Time::now();
		mosaic_image_pub_.publish(imageMsg);
	}
	else
	{
		ROS_INFO("[%s]: Some image load failed!",_NODE_NAME_);
	}
}
















