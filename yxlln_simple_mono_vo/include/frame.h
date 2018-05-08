// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#ifndef FRAME_H
#define FRAME_H

#include "common.h"
#include "camera.h"

namespace simple_vo
{

class Frame
{
public:
	double                         time_stamp_;


	bool                           is_key_frame_;

public:
	cv::Mat                        depth_;
	Camera::Ptr 				   camera_;

	cv::Mat 					   color_;
	Sophus::SE3				       T_c_w_;
	unsigned long                  id_;
	typedef std::shared_ptr<Frame> Ptr;

	Frame();
	Frame(long id, double time_stamp=0, Sophus::SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, cv::Mat clolr=cv::Mat(), cv::Mat depth=cv::Mat());
	~Frame();

	static Frame::Ptr createFrame();

	double findDepth(const cv::KeyPoint& kp);

	Eigen::Vector3d getCamCenter();

	void setPose(const Sophus::SE3& T_c_w);

	bool isInFrame(const Eigen::Vector3d& pt_world);

};

}

#endif