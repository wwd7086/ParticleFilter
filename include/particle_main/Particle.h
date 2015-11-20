/*
 * Particle.h
 *
 *  Created on: Nov 16, 2015
 *      Author: wwd
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <particle_main/Pose2D.h>

struct Particle {
	Particle(){
	}

	Particle(double x, double y, double theta) :
		pose(x,y,theta){
	}

	Pose2D pose;
	double weight=1;
};

#endif /* PARTICLE_H_ */
