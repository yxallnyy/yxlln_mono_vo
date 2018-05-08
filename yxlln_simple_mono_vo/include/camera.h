// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#ifndef CAMERA_H
#define CAMERA_H

#include "common.h"

using Sophus::SE3;

namespace simple_vo
{

class Camera
{
public:
	float fx_;
	float fy_;
	float cx_;
	float cy_;

public:
	float depth_scale_;	
    typedef std::shared_ptr<Camera> Ptr;

	Camera();

	Camera( float fx, float fy, float cx, float cy, float depth_scale=0 ):
		fx_(fx),fy_(fy),cx_(cx),cy_(cy),depth_scale_(depth_scale)
	{}

	Eigen::Vector3d world2camera(const Eigen::Vector3d& p_w, const Sophus::SE3& T_c_w);

	Eigen::Vector3d camera2world(const Eigen::Vector3d& p_c, const Sophus::SE3& T_c_w);

	Eigen::Vector3d pixel2camera(const Eigen::Vector2d p_p, double depth=1);

	Eigen::Vector2d camera2pixel(const Eigen::Vector3d);

	Eigen::Vector2d world2pixel(const Eigen::Vector3d& p_w, const Sophus::SE3& T_c_w);

	Eigen::Vector3d pixel2world(const Eigen::Vector2d& p_p, const Sophus::SE3& T_c_w, double depth=1);

};


}

#endif
