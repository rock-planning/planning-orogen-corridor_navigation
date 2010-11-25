#ifndef __CORRIDOR_SERVOING_TYPES__H
#define __CORRIDOR_SERVOING_TYPES__H

#include <vfh_star/VFH.h>
#include <base/waypoint.h>
#include <vector>

// #include <../../include/SlopeBinSegmenter.h>

namespace vfh_star {
    struct VFHStarDebugData {
	std::vector<VFHDebugData> steps;
	std::vector<base::Waypoint> generatedTrajectory;
    };   
}

#endif
