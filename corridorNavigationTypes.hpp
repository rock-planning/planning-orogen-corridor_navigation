#ifndef OROGEN_CORRIDOR_NAVIGATION_TYPES__H
#define OROGEN_CORRIDOR_NAVIGATION_TYPES__H

#include <vfh_star/VFH.h>
#include <base/waypoint.h>
#include <base/Types.hpp>
#include <vector>

namespace wrappers {
    namespace vfh_star {
        struct TreeNode {
            int parent;

            // use the marshalled version of pose from the base typekit
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

    struct FollowingDebug {
        base::Vector3d horizon[2];
        vfh_star::Tree tree;
    };
}

#endif

