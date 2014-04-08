#ifndef OROGEN_CORRIDOR_NAVIGATION_TYPES__H
#define OROGEN_CORRIDOR_NAVIGATION_TYPES__H

#include <vfh_star/VFH.h>
#include <vfh_star/TreeSearch.h>
#include <base/float.h>
#include <base/m_types/Pose.hpp>
#include <base/wrappers/Eigen.hpp>
#include <vector>
#include <corridor_planner/corridors.hh>


namespace corridor_navigation {

    struct TestConf {
        std::vector< double > angular_windows;
        double main_direction;
    };

    struct FollowingDebug {
        base::Time planning_time;
        base::Vector3d horizon[2];
        vfh_star::DebugTree tree;
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

