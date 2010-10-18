#ifndef __CORRIDOR_SERVOING_TYPES__H
#define __CORRIDOR_SERVOING_TYPES__H
#include <stdint.h>

#include <base/wrappers/eigen.h>
#include <base/wrappers/pose.h>
#include <base/wrappers/waypoint.h>
#include <base/time.h>
#ifndef __orogen
#include <vfh_star/VFH.h>
#include <vector>
#endif	
// #include <../../include/SlopeBinSegmenter.h>

namespace wrappers {

    struct VFHDebugData {
	wrappers::Pose poseb;
	std::vector< int > histogram;
	double senseRadius;
	double obstacleSafetyDist;
	double robotWidth;
#ifndef __orogen
	VFHDebugData() {};
	VFHDebugData(const vfh_star::VFHDebugData &data) {
	    poseb = data.pose;
	    for(std::vector<bool>::const_iterator it = data.histogram.begin(); it != data.histogram.end(); it++)
	    {
		histogram.push_back(*it);
	    }
	    senseRadius = data.senseRadius;
	    obstacleSafetyDist = data.obstacleSafetyDist;
	    robotWidth = data.robotWidth;
	};

	operator vfh_star::VFHDebugData() const {
	    vfh_star::VFHDebugData data;
	    for(std::vector<int>::const_iterator it = histogram.begin(); it != histogram.end(); it++)
	    {
		data.histogram.push_back(*it);
	    }
	    data.pose = poseb;
	    data.senseRadius = senseRadius;
	    data.obstacleSafetyDist = obstacleSafetyDist;
	    data.robotWidth = robotWidth;
	    return data;
	};
#endif
	
    };

    struct VFHStarDebugData {
	std::vector<VFHDebugData> steps;
	std::vector<wrappers::Waypoint> generatedTrajectory;
    };   
}

#endif
