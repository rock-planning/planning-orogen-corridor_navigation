/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TestTask.hpp"
#include <vfh_star/VFHStar.h>

using namespace corridor_navigation;
using namespace Eigen;

struct corridor_navigation::VFHStarTest : public vfh_star::VFHStar
{
AngleIntervals allowed_windows;


AngleIntervals getNextPossibleDirections(const vfh_star::TreeNode& current_node, double safetyDistance, double robotWidth) const
{

    TreeSearch::AngleIntervals result;
    double heading = current_node.getPose().getYaw();
    for (unsigned int i = 0; i < allowed_windows.size(); ++i)
    {
        double from = allowed_windows[i].first + heading;
        double to   = allowed_windows[i].second + heading;

            if (from > 2 * M_PI)
                from -= 2 * M_PI;
            else if (from < 0)
                from += 2 * M_PI;

            if (to > 2 * M_PI)
                to -= 2 * M_PI;
            else if (to < 0)
                to += 2 * M_PI;

            result.push_back( std::make_pair(from, to) );
        }

        return result;
    }

    virtual std::vector< vfh_star::ProjectedPose > getProjectedPoses(const vfh_star::TreeNode& curNode, double heading, double distance) const
    {
        std::vector< vfh_star::ProjectedPose > ret;
        Eigen::Quaterniond q = Quaterniond(AngleAxisd(heading, Vector3d::UnitZ()));
        Eigen::Vector3d p = curNode.getPose().position + q * Vector3d::UnitY() * distance;
        vfh_star::ProjectedPose pr;
        pr.pose = base::Pose(p, q);
        ret.push_back(pr);
        return ret;
    }
};

TestTask::TestTask(std::string const& name, TaskCore::TaskState initial_state)
    : TestTaskBase(name, initial_state)
    , search(new VFHStarTest)
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TestTask.hpp for more detailed
// documentation about them.

// bool TestTask::configureHook()
// {
//     if (! TestTaskBase::configureHook())
//         return false;
//     return true;
// }

bool TestTask::startHook()
{
    if (! TestTaskBase::startHook())
        return false;

    search->allowed_windows.clear();
    corridor_navigation::TestConf test_conf = _test_conf.get();
    for (unsigned int i = 0; i < test_conf.angular_windows.size() / 2; ++i)
    {
        search->allowed_windows.push_back( std::make_pair(
            test_conf.angular_windows[i * 2],
            test_conf.angular_windows[i * 2 + 1]));
    }
    
    search->setSearchConf(_search_conf.get());
    search->setCostConf(_cost_conf.get());

    return true;
}

void TestTask::updateHook()
{
    TestTaskBase::updateHook();

    base::geometry::Spline<3> trajectory =
        search->getTrajectory(_initial_pose.get(), _test_conf.get().main_direction, _search_horizon.get());
    _trajectory.write(trajectory);

    std::cerr << search->getTree().getSize() << " nodes in tree" << std::endl;
    _search_tree.write(search->getTree());
    stop();
}

// void TestTask::errorHook()
// {
//     TestTaskBase::errorHook();
// }
// void TestTask::stopHook()
// {
//     TestTaskBase::stopHook();
// }
// void TestTask::cleanupHook()
// {
//     TestTaskBase::cleanupHook();
// }

