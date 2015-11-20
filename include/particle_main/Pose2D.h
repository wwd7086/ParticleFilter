/*
 * Pose2D.h
 *
 *  Created on: Nov 18, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_PARTICLE_MAIN_POSE2D_H_
#define INCLUDE_PARTICLE_MAIN_POSE2D_H_

//std
#include <math.h>
#include <iostream>

struct Pose2D {

	Pose2D():
		x(0),y(0),theta(0){}

	Pose2D(double x, double y, double theta):
		x(x),y(y),theta(theta){
	}

	//p' = p inverse transformation
	Pose2D inv() const{
		Pose2D ip;
		ip.theta = -theta;
		ip.x = - x * cos(theta) - y * sin(theta);
		ip.y = x * sin(theta) - y * cos(theta);
		ip.normalizeRad();
		return ip;
	}

	//p12 = p1 * p2
	friend Pose2D operator*(const Pose2D& p1, const Pose2D& p2) {
		Pose2D p12;
		p12.theta = p1.theta + p2.theta;
		p12.x = p2.x * cos(p1.theta) - p2.y * sin(p1.theta) + p1.x;
		p12.y = p2.x * sin(p1.theta) + p2.y * cos(p1.theta) + p1.y;
		p12.normalizeRad();
		return p12;
	}

	//compare similarity
	bool isSimilar(const Pose2D& p) {
		const double diff = 0.001;
		if(abs(x-p.x)<diff &&
		   abs(y-p.y)<diff &&
		   abs(theta-p.theta)<diff)
			return true;
		return false;
	}

	//pretty pring
	friend std::ostream& operator<<(std::ostream& stream, const Pose2D& p) {
		stream << "{x:" << p.x
			   << " y:" << p.y
			   << " theta:" << p.theta << "}";
		return stream;
	}

	void normalizeRad(){
		while (theta < -M_PI) {
			theta += 2*M_PI;
		}
		while (theta > M_PI) {
			theta -= 2*M_PI;
		}
	}

	double x;
	double y;
	double theta;
};



#endif /* INCLUDE_PARTICLE_MAIN_POSE2D_H_ */
