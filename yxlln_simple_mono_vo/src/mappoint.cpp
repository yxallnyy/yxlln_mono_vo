// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#include "mappoint.h"


namespace simple_vo
{

Mappoint::Mappoint()
:id_(-1), pos_(Eigen::Vector3d(0,0,0)), norm_(Eigen::Vector3d(0,0,0)), good_(true), visible_times_(0),matched_times_(0)
{

}

Mappoint::Mappoint(unsigned long int id, const Eigen::Vector3d& pos, const Eigen::Vector3d& norm, Frame* frame, const cv::Mat& descriptor)
:id_(id), pos_(pos), norm_(norm), good_(true), visible_times_(1),matched_times_(1), descriptor_(descriptor)
{
	observed_frames_.push_back(frame);
}

Mappoint::Ptr Mappoint::createMapPoint()
{
	return Mappoint::Ptr(
		new Mappoint(factory_id_++, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0)));
}

Mappoint::Ptr Mappoint::createMapPoint(
		const Eigen::Vector3d& pos_world,
		const Eigen::Vector3d& norm,
		const cv::Mat& descriptor,
		Frame* frame)
{
	return Mappoint::Ptr(
		new Mappoint(factory_id_++, pos_world, norm, frame, descriptor));
}

unsigned long Mappoint::factory_id_ = 0;

}