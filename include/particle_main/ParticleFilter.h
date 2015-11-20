/*
 * ParticleFilter.h
 *
 *  Created on: Nov 16, 2015
 *      Author: wwd
 */

#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include <particle_main/Particle.h>
#include <particle_main/Map.h>
#include <particle_main/Pose2D.h>
//std
#include <vector>
#include <random>

using namespace std;

class ParticleFilter {
public:
	ParticleFilter(Map* map);
	virtual ~ParticleFilter();

	// add uniform random particle to whole map
	void addParticles(int number);
	// add uniform random particle to specific region
	void addParticlesTo(int number,
					  double xl, double xh,
					  double yl, double yh);
	void reduceParticles(int number);

	void predict(const Pose2D& lastPose, const Pose2D& curPose);
	void measure(const vector<int>& lasers,
				const Pose2D& roboPose, const Pose2D& laserPose);

	visualization_msgs::Marker visualizeParticles();
	visualization_msgs::Marker visualizeDirection();

private:
	// resample the paritcles based on its weight
	void resample(int sampleNum);
	void lowVarResample(int sampleNum);
	double weightVar();
	void normalizeWeight();

	// map and particles
	Map* map;
	vector<Particle> particles;
	int particleNum=0;

	// motion param
	default_random_engine generator;
	normal_distribution<double> dGauss;
	normal_distribution<double> rGauss;

	// sensor param
	double sensorVar=10000;//20000000;//30000;;

	// resample criteria
	double minmaxThre=1000; //550
	double varianceThre=0.2;
};

#endif /* PARTICLEFILTER_H_ */
