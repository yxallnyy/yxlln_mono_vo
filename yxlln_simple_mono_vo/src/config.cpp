// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#include "config.h"

namespace simple_vo
{

void Config::setParameterFile(const std::string& filename)
{
	if(config_ == nullptr)
	{
		config_ = std::shared_ptr<Config>(new Config);
	}
	config_->file_ =cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
	if(!config_->file_.isOpened())
	{
		cout<<"parameter file does not exist!!!!"<<endl;
		config_->file_.release();
		return;
	}
}

Config::~Config()
{
	if(config_->file_.isOpened())
	{
		config_->file_.release();
	}
}

std::shared_ptr<Config> Config::config_ = nullptr;

}