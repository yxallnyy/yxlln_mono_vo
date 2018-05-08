// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

namespace simple_vo
{

class Config
{
private:

	cv::FileStorage file_;

public:
	static std::shared_ptr<Config> config_;
	Config(){}
	~Config();

	static void setParameterFile(const std::string& filename);

	template<typename T>
	static T get(const std::string& key)
	{
		return T(Config::config_->file_[key]);
	}

};

}

#endif