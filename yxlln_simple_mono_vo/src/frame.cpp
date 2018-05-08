// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#include "frame.h"

namespace simple_vo
{

Frame::Frame():id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

Frame::Frame(long id, double time_stamp, Sophus::SE3 T_c_w, Camera::Ptr camera, cv::Mat color, cv::Mat depth)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
	static long factory_id = 0;
	return Frame::Ptr(new Frame(factory_id++));
}

double Frame::findDepth(const cv::KeyPoint& kp)
{
	int x = cvRound(kp.pt.x);
	int y = cvRound(kp.pt.y);
	ushort d = depth_.ptr<ushort>(y)[x];
	if(d!=0)
	{
		return double(d)/camera_->depth_scale_;
	}
	else
	{
		int dx[4] = {-1,0,1,0};
		int dy[4] = {0,01,0,1};
		for(int i=0;i<4;i++)
		{
			d = depth_.ptr<ushort>(y+dy[i])[x+dx[i]];
			if(d!=0)
			{
				return double(d)/camera_->depth_scale_;
			}
		}
	}
	return -1.0;
}
//	void setPose(const Sophus::SE3 T_c_w);
void Frame::setPose(const Sophus::SE3& T_c_w)
{
	T_c_w_ = T_c_w;
}

Eigen::Vector3d Frame::getCamCenter()
{
	return T_c_w_.inverse().translation();
}

bool Frame::isInFrame(const Eigen::Vector3d& pt_world)
{
	Eigen::Vector3d p_cam = camera_->world2camera(pt_world, T_c_w_);
	if(p_cam(2,0)<0)
	{
		return false;
	}
	Eigen::Vector2d p_pixel = camera_->world2pixel(pt_world,T_c_w_);
	return p_pixel(0,0)>0 && p_pixel(1,0)>0
		&& p_pixel(0,0)<color_.cols
		&& p_pixel(1,0)<color_.rows;
}


}