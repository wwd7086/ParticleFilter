/*
 * Map.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: wwd
 */

#include <particle_main/Map.h>
//std
#include <stdio.h>

Map::Map() {
	// TODO Auto-generated constructor stub

}

Map::~Map() {
	delete[] map;
}

float Map::at(double x, double y) {
	pair<int,int> ind = getIndex(x,y);
	return get(ind.first, ind.second);
}

pair<double,double> Map::closeCenter(double x, double y) {
	pair<int,int> ind = getIndex(x,y);
	return make_pair((ind.first+0.5)*resolution,
			(ind.second+0.5)*resolution);
}

pair<int, int> Map::getIndex(double x, double y){
	int i = floor(x/resolution);
	int j = floor(y/resolution);

	if (i < 0)
		i = 0;
	if (i >= sizeX)
		i = sizeX - 1;
	if (j < 0)
		j = 0;
	if (j >= sizeY)
		j = sizeY - 1;

	return make_pair(i,j);
}

void Map::set(int i, int j, float val) {
	map[i * sizeY + j] = val;
}

float Map::get(int i, int j) {
	return map[i * sizeY + j];
}

int Map::getSizeX() {
	return sizeX;
}

int Map::getSizeY() {
	return sizeY;
}

int Map::getResolution(){
	return resolution;
}

bool Map::readMap(string fileName) {

	const char* mapName = fileName.c_str();
	char line[256];
	FILE *fp;

	// open the map file
	if ((fp = fopen(fileName.c_str(), "rt")) == NULL) {
		fprintf(stderr, "# Could not open file %s\n", mapName);
		return false;
	}
	fprintf(stderr, "# Reading map: %s\n", mapName);
	// read parameters
	while ((fgets(line, 256, fp) != NULL)
			&& (strncmp("global_map[0]", line, 13) != 0)) {
		if (strncmp(line, "robot_specifications->resolution", 32) == 0) {
			if (sscanf(&line[32], "%d", &(this->resolution)) != 0)
				printf("# Map resolution: %d cm\n", this->resolution);
		}
		if (strncmp(line, "robot_specifications->autoshifted_x", 35) == 0) {
			if (sscanf(&line[35], "%g", &(this->offX)) != 0) {
				printf("# Map offsetX: %g cm\n", this->offX);
			}
		}
		if (strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
			if (sscanf(&line[35], "%g", &(this->offY)) != 0) {
				printf("# Map offsetY: %g cm\n", this->offY);
			}
		}
	}
	// read size
	if (sscanf(line, "global_map[0]: %d %d", &(this->sizeY), &(this->sizeX))
			!= 2) {
		fprintf(stderr, "ERROR: corrupted file %s\n", mapName);
		fclose(fp);
		return false;
	}
	printf("# Map size: %d %d\n", sizeX, sizeY);
	// create map based on size
	map = new float[sizeX * sizeY];

	// fill in the map
	int count = 0, x = 0, y = 0;
	float temp = 0;
	for (x = 0; x < sizeX; x++) {
		for (y = 0; y < sizeY; y++, count++) {
			if (count % 10000 == 0) {
				fprintf(stderr, "\r# Reading ... (%.2f%%)",
						count / (float) (sizeX * sizeY) * 100);
			}

			fscanf(fp, "%e", &temp);

			if (temp < 0.0)
				set(x, y, -1);
			else
				set(x, y, 1 - temp);
		}
	}
	fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",
			count / (float) (sizeX * sizeY) * 100);
	fclose(fp);
	return true;
}

visualization_msgs::Marker Map::visualize() {

	visualization_msgs::Marker marker;
	//basic info
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	// set cube size
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0;
	// set pose
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.color.a = 1.0;

	int x = 0, y = 0;
	for (x = 0; x < sizeX; x++) {
		for (y = 0; y < sizeY; y++) {
			// set cube location
			geometry_msgs::Point p;
			p.x=x*0.1+0.05; p.y=y*0.1+0.05; p.z=0;
			marker.points.push_back(p);
			// set cube color
			std_msgs::ColorRGBA c;
			c.a=0.2; c.r=0; c.g=0; c.b=0;
			float occu = get(x,y);
			if(occu<0) {
				c.b =0.5;
			} else {
				c.g = occu;
			}

			marker.colors.push_back(c);
		}
	}

	return marker;

}

