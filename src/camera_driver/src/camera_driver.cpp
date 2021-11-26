
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/detail/distortion_model.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <algorithm>
#include <ros/ros.h>
#include <string>


#define _NODE_NAME_ "camera_driver_node"

using namespace std;

class ImageTalker
{
public:
	ImageTalker()
	{
	}
	bool init()
	{
		ros::NodeHandle nh,nh_private("~");
		
		std::string calibrationFilePath = nh_private.param<std::string>("calibration_file_path","");
		
		if(calibrationFilePath.empty())
		{
			ROS_ERROR("[%s]: please set calibration_file_path parameter",_NODE_NAME_);
			return false;
		}
		
		nh_private.param<bool>("is_show_result",m_isShowResult,false);
		nh_private.param<bool>("is_show_alone_image",m_isShowAloneImage,false);
		
		nh_private.param<int>("frame_rate",m_frameRate,30);
		
		if(!ros::param::get("~cameras_soft_id",m_camerasSoftId))
		{
			ROS_ERROR("[%s]: please set cameras_soft_id parameter",_NODE_NAME_);
			return false;
		}
		if(!ros::param::get("~cameras_hard_id",m_camerasHardId))
		{
			ROS_ERROR("[%s]: please set cameras_hard_id parameter",_NODE_NAME_);
			return false;
		}
		
		// Support multiple cameras with multiple calibration files working at the same time.
		// If multiple cameras are used, setup cameras_soft_id and cameras_hard_id in launch file in the form of vector.
		// Parameter cameras_soft_id represents camera number in the computer.
		// Parameter cameras_hard_id represents ID of calibration file.
		m_distortCoefficients.resize(m_camerasSoftId.size());
		m_instrinsics.resize(m_camerasSoftId.size());
		m_newInstrinsics.resize(m_camerasSoftId.size());
		m_cameraHandles.resize(m_camerasSoftId.size());
		m_imagePublisher.resize(m_camerasSoftId.size());
		m_rawImages.resize(m_camerasSoftId.size());
		m_distMap.resize(m_camerasSoftId.size());
		m_rectifiedImages.resize(m_camerasSoftId.size());
		m_imageGrapFlags = std::vector<bool>(m_camerasSoftId.size(),false);
		m_mutexes.reset(new std::vector<boost::mutex>(m_camerasSoftId.size()));
		m_conditionVars.reset(new std::vector<boost::condition_variable>(m_camerasSoftId.size()));
		image_transport::ImageTransport it(nh);

		for(int i=0; i<m_camerasSoftId.size(); ++i)
		{
			std::string file_name = calibrationFilePath + std::to_string(m_camerasHardId[i]) + ".yaml";
			if(!loadIntrinsics(file_name,m_instrinsics[i],m_distortCoefficients[i]))
				return false;
			
			m_newInstrinsics[i] = getOptimalNewCameraMatrix(m_instrinsics[i],m_distortCoefficients[i],m_imgSize,0.0);
			
			cv::initUndistortRectifyMap(m_instrinsics[i], m_distortCoefficients[i], cv::Mat(),
									m_newInstrinsics[i], m_imgSize, CV_16SC2, m_distMap[i].first, m_distMap[i].second);
			if(!m_cameraHandles[i].open(m_camerasSoftId[i]))
			{
				ROS_ERROR("[%s] open camera %d failed",_NODE_NAME_,m_camerasSoftId[i]);
				return false;
			}
			m_cameraHandles[i].set(CV_CAP_PROP_FPS,m_frameRate);
			m_cameraHandles[i].set(CV_CAP_PROP_FRAME_WIDTH,m_imgSize.width);
			m_cameraHandles[i].set(CV_CAP_PROP_FRAME_HEIGHT,m_imgSize.height);
			
			std::string image_topic_name = std::string("/image_rect") + std::to_string(m_camerasHardId[i]);
			m_imagePublisher[i] = it.advertise(image_topic_name, 1);
		}
	}
	
	void imageDistortThread(size_t index)
	{
		while(ros::ok())
		
		{
			boost::unique_lock<boost::mutex> locker((*m_mutexes)[index]);
			
			while(!m_imageGrapFlags[index])
				(*m_conditionVars)[index].wait(locker);
			
			m_cameraHandles[index].retrieve(m_rawImages[index]);
			m_imageGrapFlags[index] = false;
			locker.unlock();
			
			cv::remap(m_rawImages[index], m_rectifiedImages[index], m_distMap[index].first, m_distMap[index].second, cv::INTER_LINEAR);
			sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_rectifiedImages[index]).toImageMsg();
			imageMsg->header.frame_id = std::string("camera") + std::to_string(index);
			imageMsg->header.stamp = ros::Time::now();
			m_imagePublisher[index].publish(imageMsg);
		}
	}
	
	void showImageThread()
	{
		boost::this_thread::sleep_for(boost::chrono::seconds(1));
		
		for(size_t i=0; i<m_camerasHardId.size(); ++i)
			cv::namedWindow(std::string("image_rect")+std::to_string(m_camerasHardId[i]), cv::WINDOW_NORMAL);
	
		while(ros::ok())
		{
			for(size_t i=0; i<m_camerasHardId.size(); ++i)
			{
				(*m_mutexes)[i].lock();
				cv::imshow(std::string("image_rect")+std::to_string(m_camerasHardId[i]), m_rectifiedImages[i]);
				(*m_mutexes)[i].unlock();
			}
			cv::waitKey(50);
		}
		cv::destroyAllWindows();
	}
	
	void run()
	{
		std::vector<boost::thread> _thread(m_camerasSoftId.size());
		for(int i=0; i<m_camerasSoftId.size(); ++i)
			_thread[i] = boost::thread(boost::bind(&ImageTalker::imageDistortThread,this,i));
		if(m_isShowAloneImage)
			boost::thread t = boost::thread(boost::bind(&ImageTalker::showImageThread,this));
		
		ros::Rate loop_rate(m_frameRate);
		while(ros::ok())
		{
			for(int i=0; i<m_cameraHandles.size(); ++i)
			{
				boost::unique_lock<boost::mutex> locker((*m_mutexes)[i]);
				m_cameraHandles[i].grab();
				m_imageGrapFlags[i] = true;
				(*m_conditionVars)[i].notify_one();
			}
			
			loop_rate.sleep();
		}
	}
	
	bool loadIntrinsics(const std::string &calibration_file, cv::Mat &camera_instrinsic, cv::Mat &distortion_coefficients)
	{
		if (calibration_file.empty())
		{
			ROS_ERROR("[%s] missing calibration file path", _NODE_NAME_);
			return false;
		}

		cv::FileStorage fs(calibration_file, cv::FileStorage::READ);

		if (!fs.isOpened())
		{
			ROS_INFO("[%s] cannot open calibration file %s", _NODE_NAME_, calibration_file.c_str());
	 		return false;
		}

		camera_instrinsic = cv::Mat(3, 3, CV_64F);
		distortion_coefficients = cv::Mat(1, 5, CV_64F);

		cv::Mat dis_tmp;
		
		fs["CameraMat"] >> camera_instrinsic;
		fs["DistCoeff"] >> dis_tmp;
		fs["ImageSize"] >> m_imgSize;
		fs["DistModel"] >> m_distModel;
		
		for (int col = 0; col < 5; col++)
			distortion_coefficients.at<double>(col) = dis_tmp.at<double>(col);
		fs.release();
		return true;
	}

private:
	std::vector<image_transport::Publisher> m_imagePublisher;
	std::vector<int> m_camerasSoftId;
	std::vector<int> m_camerasHardId;
	std::vector<cv::Mat> m_newInstrinsics;
	std::vector<cv::Mat> m_instrinsics;
	std::vector<cv::Mat> m_distortCoefficients;
	std::vector<cv::VideoCapture> m_cameraHandles;
	std::vector<cv::Mat> m_rawImages;
	std::vector<std::pair<cv::Mat,cv::Mat> > m_distMap;
	std::vector<cv::Mat> m_rectifiedImages;
	std::vector<bool> m_imageGrapFlags;
	std::unique_ptr<std::vector<boost::mutex> > m_mutexes;
	std::unique_ptr<std::vector<boost::condition_variable> > m_conditionVars;
	
	std::string m_distModel;
	cv::Size m_imgSize;
	int m_frameRate;
	bool m_isShowResult;
	bool m_isShowAloneImage;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	
	ImageTalker image_talker;
	if(image_talker.init())
		image_talker.run();
	return 0;
}
