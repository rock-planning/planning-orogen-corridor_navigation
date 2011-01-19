#ifndef __CORRIDOR_SERVOING_TYPES__H
#define __CORRIDOR_SERVOING_TYPES__H

#include <vfh_star/VFH.h>
#include <base/waypoint.h>
#include <base/Types.hpp>
#include <vector>

namespace wrappers {
    namespace vfh_star {
        struct TreeNode {
            int parent;

            base::Pose_m pose;
            double cost;
            double heuristic;
            double direction;
            double positionTolerance;
            double headingTolerance;
        };

        struct Tree {
            std::vector< TreeNode > nodes;
            int finalNode;
        };
    }
}

namespace corridor_navigation {
    struct TestConf {
        std::vector< double > angular_windows;
        double main_direction;
    };
}

#endif

