/*
 * ParticleFilter.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: wwd
 */

#include <particle_main/ParticleFilter.h>
//std
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>

inline double randNum(double l, double h) {
	return (((double) rand()) / RAND_MAX) * (h - l) + l;
}

inline double limit(double val, double l, double h) {
	if (val < l)
		val = l;
	else if (val > h)
		val = h;

	return val;
}

ParticleFilter::ParticleFilter(Map* map):
		dGauss(0, 1),
		rGauss(0, 0.03) {

	this->map = map;
	particles.clear();
	srand(time(NULL));
}

ParticleFilter::~ParticleFilter() {
	// TODO Auto-generated destructor stub
}

void ParticleFilter::addParticles(int number) {
	cout << "add particles:" << number << endl;

	double tx = map->getSizeX() * map->getResolution();
	double ty = map->getSizeY() * map->getResolution();

	addParticlesTo(number, 0, tx, 0, ty);
}

void ParticleFilter::addParticlesTo(int number,
		double xl, double xh,
		double yl, double yh) {

	int i = 0;
	while (i < number) {
		double x = randNum(xl, xh);
		double y = randNum(yl, yh);
		double rad = randNum(-M_PI, M_PI);

		double occup = map->at(x, y);
		if (occup == 0) {
			particles.push_back(Particle(x, y, rad));
			i++;
		}
	}

	particleNum += number;
}

void ParticleFilter::reduceParticles(int number) {
	lowVarResample(number);
}

void ParticleFilter::predict( const Pose2D& lastPose, const Pose2D& curPose) {

	// calculate delta movement based on two odometry frame
	Pose2D deltaPose = lastPose.inv() * curPose;

	for (auto it = particles.begin(); it != particles.end(); it++) {

		// add gaussian noise
		Pose2D dp = deltaPose;
		dp.x += dGauss(generator);
		dp.y += dGauss(generator);
		dp.theta += rGauss(generator);

		// increment particle state
		it->pose = it->pose * dp;

		// normalize rad
		it->pose.normalizeRad();

		// check boundary
		it->pose.x = limit(it->pose.x, 0, 8000);
		it->pose.y = limit(it->pose.y, 0, 8000);
	}
}

void ParticleFilter::measure(const vector<int>& lasers,
		const Pose2D& roboPose, const Pose2D& laserPose) {

	//ray tracing
	double minWeight = std::numeric_limits<double>::max();
	double maxWeight = 0;

	Pose2D rtol = roboPose.inv() * laserPose;
	for (auto it = particles.begin(); it != particles.end(); it++) {
		// sensor in world coordinate
		Pose2D laserWorld = it->pose * rtol;
		laserWorld.normalizeRad();
		// for each angle
		double diff = 0;
		const double dr = M_PI / 180;
		double rad = laserWorld.theta - M_PI / 2 + 0.5*dr;
		for (int r = 0; r < 180; r+=10) {
			// seach along line
			int l = 0;
			double dx = cos(rad), dy = sin(rad);
			for (; l < 8500; l += 10) {
				double x = laserWorld.x + l * dx;
				double y = laserWorld.y + l * dy;

				double occu = map->at(x, y);
				if (occu < 0 || occu >= 0.1) {
					//refine l
					break;
				}
			}
			rad += (dr*10);
			// compare to measurement
			//diff += pow(l - lasers[r], 2);  //l2
			diff += abs(l-lasers[r]);     //l1
		}
		// update the weight for each particle
		diff = pow(2.7, -(diff / sensorVar));
		it->weight *= diff;

		// book keeping min max
		if (it->weight > maxWeight)
			maxWeight = it->weight;
		else if (it->weight < minWeight)
			minWeight = it->weight;
	}

	// resample if needed
	//double variance = weightVar();
	cout << "min/max:" << maxWeight / minWeight << endl;
	//cout << "variance" << variance << endl;
	if (maxWeight / minWeight > minmaxThre) {
		//&& variance > varianceThre) {
		cout << "resample" << endl;
		lowVarResample(particleNum);
	}
}

void ParticleFilter::resample(int sampleNum) {

	//build sample map
	double weightSum = 0;
	vector<pair<double, int>> weights;
	weights.reserve(particleNum);
	int count = 0;
	for (auto it = particles.begin(); it != particles.end(); it++) {
		weightSum += it->weight;
		weights.push_back(make_pair(it->weight, count++));
	}
	sort(weights.rbegin(), weights.rend()); //sort for efficiency

	//resample
	vector<Particle> newParticles;
	newParticles.reserve(sampleNum);
	for (int i = 0; i < sampleNum; i++) {
		//sample from discrete distirbution
		double ran = randNum(0, weightSum);
		double weightAccu = 0;
		auto it = weights.begin();
		for (; it != weights.end(); it++) {
			weightAccu += it->first;
			if (weightAccu >= ran)
				break;
		}
		//add particle
		particles[it->second].weight = 1;
		newParticles.push_back(particles[it->second]);
	}

	//swap
	particles = newParticles;
	particleNum = sampleNum;
}

// low variance resampling
void ParticleFilter::lowVarResample(int sampleNum) {
	// build up all the weights
	double weightSum = 0;
	vector<double> weights;
	weights.reserve(particleNum);

	for(auto particle:particles) {
		weightSum += particle.weight;
		weights.push_back(particle.weight);
	}

	// sample new paritcle
	vector<Particle> newParticles;
	double dw = weightSum/sampleNum;
	double adw = randNum(0,dw);
	double aw = weights[0];
	int wind = 0;
	for(int i=0; i<sampleNum; i++) {
		while(adw>aw) {
			aw += weights[++wind];
		}
		particles[wind].weight = 1;
		newParticles.push_back(particles[wind]);
		adw += dw;
	}

	//swap
	particles = newParticles;
	particleNum = sampleNum;
}

// calculate the variance of weights of all particels
double ParticleFilter::weightVar() {
	// calculate mean
	double weightSum=0;
	for(auto particle:particles) {
		weightSum += particle.weight;
	}
	double mean = weightSum / particleNum;
	// calculate var
	double variance = 0;
	for(auto particle:particles) {
		variance += pow(particle.weight - mean,2);
	}
	return variance/particleNum;
}

// normalize weight so they sum up to number of pariticles
void ParticleFilter::normalizeWeight(){

}

visualization_msgs::Marker ParticleFilter::visualizeParticles() {
	visualization_msgs::Marker marker;
	//basic info
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time();
	marker.ns = "basic_shapes";
	marker.id = 1;
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
	// set color
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0;
	marker.color.b = 0;

	for (auto it = particles.begin(); it != particles.end(); it++) {
		// set cube location
		geometry_msgs::Point p;
		p.x = it->pose.x / 100;
		p.y = it->pose.y / 100;
		p.z = 0;
		marker.points.push_back(p);
	}

	return marker;
}

visualization_msgs::Marker ParticleFilter::visualizeDirection() {

	visualization_msgs::Marker marker;
	//basic info
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time();
	marker.ns = "basic_shapes";
	marker.id = 2;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	// set line width
	marker.scale.x = 0.01;
	// set color
	marker.color.a = 1.0;
	marker.color.r = 0;
	marker.color.g = 0;
	marker.color.b = 1.0;

	for (auto it = particles.begin(); it != particles.end(); it++) {
		// set point start
		geometry_msgs::Point p;
		p.x = it->pose.x / 100;
		p.y = it->pose.y / 100;
		p.z = 0;
		marker.points.push_back(p);
		// set point end
		p.x += cos(it->pose.theta)/10;
		p.y += sin(it->pose.theta)/10;
		marker.points.push_back(p);
	}

	return marker;
}

