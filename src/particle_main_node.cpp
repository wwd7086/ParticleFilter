//============================================================================
// Name        : particle_filter.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <particle_main/Map.h>
#include <particle_main/ParticleFilter.h>
//std
#include <iostream>
#include <fstream>

using namespace std;

const int INIT_PARTICLES = 10000;
const int RED_PARTICLES = 3000;

int main(int argc, char** argv) {
	string fileRootName = "/home/wenda/Google Drive/Developer/stat/project4/data/";
	string mapFileName = fileRootName + "map/wean.dat";
	string measureFileName = fileRootName + "log/robotdata1.log";

	// init ros
	ros::init(argc, argv, "particle_filter");
	ros::NodeHandle n;
	ros::Publisher mark_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	while (!ros::ok()) {
	}

	// init map
	Map map;
	// load the map
	map.readMap(mapFileName);
	// show the map
	mark_pub.publish(map.visualize());

	// init filter
	ParticleFilter filter(&map);
	filter.addParticles(INIT_PARTICLES);

	// load measurment file
	ros::Rate r(20);
	bool isFirst = true;
	int measureCount=0;
	ifstream measureFile(measureFileName);
	if (measureFile.is_open()) {
		string line;
		Pose2D lastrpose;
		while (getline(measureFile, line)) {
			// read measure
			istringstream iss(line);
			vector<string> tokens { istream_iterator<string> { iss },
					istream_iterator<string> { } };

			cout << "-------------round "<<measureCount++ <<"---------------" << endl;

			// reduce sample
			if(measureCount == 200) {
				filter.reduceParticles(RED_PARTICLES);
			}

			// process measure
			if (tokens[0] == "L") {
				//Laser data
				Pose2D rpose(stod(tokens[1]), stod(tokens[2]), stod(tokens[3]));
				Pose2D lpose(stod(tokens[4]), stod(tokens[5]), stod(tokens[6]));
				vector<int> lasers;
				lasers.reserve(180);
				for (int i = 7; i < 187; i++) {
					lasers.push_back(stoi(tokens[i]));
				}

				//info
				cout << "reading laser:" << endl;
				cout << "r:" << rpose << "l:" << lpose << endl;

				//run filter
				if (isFirst) {
					isFirst = false;
					lastrpose = rpose;
				} else {
					filter.predict(lastrpose, rpose);
					if(!lastrpose.isSimilar(rpose))
						filter.measure(lasers, rpose, lpose);
					lastrpose = rpose;
				}

			} else if (tokens[0] == "O") {
				//odometry data
				Pose2D rpose(stod(tokens[1]), stod(tokens[2]), stod(tokens[3]));

				//info
				cout << "reading Odometry:" << endl;
				cout << "r:" << rpose << endl;

				//run filter
				if(isFirst) {
					isFirst = false;
					lastrpose = rpose;
				} else {
					filter.predict(lastrpose, rpose);
					lastrpose = rpose;
				}
			}

			// visualize resulting particless
			mark_pub.publish(filter.visualizeParticles());
			mark_pub.publish(filter.visualizeDirection());

			r.sleep();
		}
	}

	ros::shutdown();

	return 0;
}
