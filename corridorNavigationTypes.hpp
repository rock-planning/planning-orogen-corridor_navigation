#ifndef OROGEN_CORRIDOR_NAVIGATION_TYPES__H
#define OROGEN_CORRIDOR_NAVIGATION_TYPES__H

#include <vfh_star/VFH.h>
#include <vfh_star/TreeSearch.h>
#include <base/float.h>
#include <base/m_types/Pose.hpp>
#include <base/wrappers/Eigen.hpp>
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
            //Position and orientation of the tree itself
            base::Pose_m tree2World;
            int finalNode;
        };
    }
}

namespace corridor_navigation {

    struct SweepStatus
    {
        enum State {
            NOT_SWEEPING,
            SWEEPING_UP,
            SWEEPING_DOWN,
        };
        ///counter of the sweeps, wraps at 255
        uint8_t counter;
        
        ///current state of sweeping
        State curState;
        
        ///name of the sweeping device
        std::string sourceName;
        
        /**
         * Return true if one ore more sweeps
         * in respect to the given state werde done.
         * 
         * Also returns true, if not sweeping at all.
         * */
        bool isNextSweep(SweepStatus lastState)
        {
            return NOT_SWEEPING || lastState.counter != counter;
        }
    };

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

