/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FollowingTask.hpp"
#include <corridor_navigation/VFHFollowing.hpp>

using namespace corridor_navigation;

FollowingTask::FollowingTask(std::string const& name, TaskCore::TaskState initial_state)
    : FollowingTaskBase(name, initial_state)
    , search(new corridor_navigation::VFHFollowing)
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FollowingTask.hpp for more detailed
// documentation about them.

// bool FollowingTask::configureHook()
// {
//     if (! FollowingTaskBase::configureHook())
//         return false;
//     return true;
// }
bool FollowingTask::startHook()
{
    if (! FollowingTaskBase::startHook())
        return false;

    search->setSearchConf(_search_conf.get());
    search->setCostConf(_cost_conf.get());
    return true;
}

void FollowingTask::updateHook()
{
    FollowingTaskBase::updateHook();

    corridors::Corridor corridor;
    if (_corridor.readNewest(corridor) == RTT::NewData)
    {
        search->setCorridor(corridor);
    }

    base::samples::RigidBodyState current_pose;
    if (_pose_samples.readNewest(current_pose) == RTT::NoData)
        return;
    try
    {
        base::Time start = base::Time::now();
        std::cerr << "starting" << std::endl;
        base::geometry::Spline<3> trajectory =
            search->getTrajectory(base::Pose(current_pose.position, current_pose.orientation), _search_horizon.get());
        std::cerr << "planning took " << (base::Time::now() - start).toMilliseconds() << " milliseconds" << std::endl;

        _trajectory.write(trajectory);

        outputDebuggingTypes();
    }
    catch(std::exception const& e)
    {
        outputDebuggingTypes();
        throw;
    }

}

void FollowingTask::outputDebuggingTypes()
{
    if (_debugVfhTree.connected())
        _debugVfhTree.write(search->getTree());
}

// void FollowingTask::errorHook()
// {
//     FollowingTaskBase::errorHook();
// }
// void FollowingTask::stopHook()
// {
//     FollowingTaskBase::stopHook();
// }
// void FollowingTask::cleanupHook()
// {
//     FollowingTaskBase::cleanupHook();
// }

