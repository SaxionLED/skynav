/*
 * file_reader.cpp
 *
 *  Created on: Dec 5, 2013
 *      Author: jurriaan
 */

#include <ros/ros.h> // You must include this to do things with ROS.
#include <ros/node_handle.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <ios>

#include <skynav_msgs/mapreader_srv.h>

const int LOOP_RATE = 1;
const int NO_LOOP = 0;

const int OCCUPIED = 100;
const int CFREE = 0;

struct Map_Info {
	float mapHight;
	float mapWidth;
	float resolution;
	std::vector<int8_t> mOccupancydata;
	Map_Info() {

	}
};

class Map_Reader {
private:
	std::string node_name_;
	ros::NodeHandle* node_;
	int loop_rate_;

	//service
	ros::ServiceServer read_map_serv_;

	Map_Info* p_mMap_Info;

public:
	Map_Reader(std::string node_name);
	virtual ~Map_Reader() {
		delete p_mMap_Info;
		delete node_;
	}
	;

	void loop();
	bool respond(skynav_msgs::mapreader_srv::Request &req,
			skynav_msgs::mapreader_srv::Response &res);
	bool parseMap(std::string filepath);

	Map_Info* getMapData() const;
};

Map_Reader::Map_Reader(std::string node_name) :
		node_name_(node_name) {

	node_ = new ros::NodeHandle("/SLAM");

	read_map_serv_ = node_->advertiseService("map_read_req",&Map_Reader::respond, this);

	p_mMap_Info = new Map_Info();
}

bool Map_Reader::respond(skynav_msgs::mapreader_srv::Request &req,
		skynav_msgs::mapreader_srv::Response &res) {
	if (req.request) {
		ROS_INFO("received map read request");
		if (parseMap(req.file_path)) {
			res.response = 1;
			res.map.info.height = this->getMapData()->mapHight;
			res.map.info.width = this->getMapData()->mapWidth;
			res.map.info.resolution = this->getMapData()->resolution;
			res.map.data.clear();
			for(int i = 0; i<this->getMapData()->mOccupancydata.size(); i++){
				res.map.data.push_back(this->getMapData()->mOccupancydata[i]);
			}
			return true;
		}
	}
	ROS_ERROR("No map could be read");
	return false;
}

bool Map_Reader::parseMap(std::string file_path) {
	ROS_INFO("parsing map %s", file_path.c_str());

	std::ifstream file(file_path.c_str());
	file.unsetf(std::ios_base::skipws);
	std::vector<char> tempArray;
	if (file.good()) {
		p_mMap_Info->mOccupancydata.clear();
		int xDim = 0;
		int yDim = 0;

		for (char c; file >> c;) {
			tempArray.push_back(c);
		}

		std::vector<int8_t> data;
		int object = OCCUPIED;	//integer value for gridcell occupied
		int empty = CFREE;		//integer value for gridcell empty
		for (int i = 0; i < tempArray.size(); i++) {
			if (tempArray[i] == '#') {
				data.push_back(object);
			} else if (tempArray[i] == '\n') {
				yDim += 1;
			} else {
				data.push_back(empty);

			}
		}
		xDim = data.size() / yDim;

		this->p_mMap_Info->mapWidth = xDim;
		this->p_mMap_Info->mapHight = yDim;
		//TODO resolution!! default = 1
		this->p_mMap_Info->resolution = 1;

		this->p_mMap_Info->mOccupancydata = data;

		data.clear();
		tempArray.clear();

		ROS_INFO("parsing map done");
		return true;
	}
	ROS_ERROR("file %s does not exist", file_path.c_str());
	return false;
}

Map_Info* Map_Reader::getMapData() const {
	return p_mMap_Info;
}

void Map_Reader::loop() {
	ros::spin();
}

int main(int argc, char ** argv) {
	std::string node_name = "map_reader";
	ros::init(argc, argv, node_name);
	
	ROS_INFO("started map_reader");
	Map_Reader x(node_name);
	x.loop();
	ROS_INFO("map_reader exit");
	return 0;
}
