#include <ros/ros.h>
#include "rapidxml.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <streambuf>
#include <iostream>

int main(int argc, char* argv[]) {

	std::ifstream inp_xml_file("sample.xml");
	std::string inp_xml((std::istreambuf_iterator<char>(inp_xml_file)), 
						std::istreambuf_iterator<char>());


	std::cout << inp_xml;


	ros::init(argc, argv, "uav_master");
	ros::NodeHandle nh;

	return 0;
}
