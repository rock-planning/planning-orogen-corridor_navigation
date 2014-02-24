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
    base::Angle heading = base::Angle::fromRad(current_node.getPose().getYaw());
    for (unsigned int i = 0; i < allowed_windows.size(); ++i)
    {
        const base::AngleSegment &cur(allowed_windows[i]);
        result.push_back(base::AngleSegment(cur.getStart() + heading, cur.getWidth()));
    }
    return result;
}

    virtual std::vector< vfh_star::ProjectedPose > getProjectedPoses(const vfh_star::TreeNode& curNode, const base::Angle& heading, double distance) const
    {
        std::vector< vfh_star::ProjectedPose > ret;
        Eigen::Quaterniond q = Quaterniond(AngleAxisd(heading.getRad(), Vector3d::UnitZ()));
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
    for (unsigned int i = 0; i < test_conf.angular_windows.size(); i += 2)
    {
        double from = test_conf.angular_windows[i];
        double to = test_conf.angular_windows[i + 1];
        search->allowed_windows.push_back( base::AngleSegment(base::Angle::fromRad(from), to - from));
    }
    
    search->setSearchConf(_search_conf.get());
    search->setCostConf(_cost_conf.get());

    return true;
}

void TestTask::updateHook()
{
    TestTaskBase::updateHook();
    std::vector<base::Trajectory> trajectories = search->getTrajectories(_initial_pose.get(), base::Angle::fromRad(_test_conf.get().main_direction), _search_horizon.get());
    if(trajectories.size())
        _trajectory.write(trajectories.begin()->spline);

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

