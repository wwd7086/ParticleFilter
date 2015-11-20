/*
 * Map.h
 *
 *  Created on: Nov 16, 2015
 *      Author: wwd
 */

#ifndef MAP_H_
#define MAP_H_

//std
#include <string>
//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class Map {
public:
	Map();
	virtual ~Map();
	bool readMap(string fileName);
	visualization_msgs::Marker visualize();

	// query by world coordinate
	float at(double x, double y);
	pair<double, double> closeCenter(double x, double y);
	// query by grid index
	void set(int i, int j, float val);
	float get(int i, int j);

	int getSizeX();
	int getSizeY();
	int getResolution();

private:
	pair<int,int> getIndex(double x, double y);
	int sizeX = 0;
	int sizeY = 0;
	float offX = 0;
	float offY = 0;
	int resolution = 10;
	float* map=0;
};

#endif /* MAP_H_ */
