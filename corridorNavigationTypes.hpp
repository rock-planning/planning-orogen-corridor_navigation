#ifndef OROGEN_CORRIDOR_NAVIGATION_TYPES__H
#define OROGEN_CORRIDOR_NAVIGATION_TYPES__H

#include <vfh_star/VFH.h>
#include <base/float.h>
// Load the base typekit to get Pose_m, used to create the vfh_star::TreeNode
// wrapper
#include <base/Types.hpp>
#include <vector>
#include <corridor_planner/corridors.hh>

namespace wrappers {
    namespace vfh_star {
        struct TreeNode {
            //id for tree flattening and reconstruction
            int nodeId;

            // use the marshalled version of pose from the base typekit
            base::Pose_m pose;
            double cost;
            double heuristic;
            double direction;
            double positionTolerance;
            double headingTolerance;
            std::vector< int > childs;
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
        base::Time planning_time;
        base::Vector3d horizon[2];
        vfh_star::Tree tree;
    };

    /** Type used to provide a complete problem to the task
     */
    struct CorridorFollowingProblem {
        double desiredFinalHeading;
        corridors::Corridor corridor;

        CorridorFollowingProblem()
            : desiredFinalHeading(base::unset<double>()) {}
    };
}

#endif

