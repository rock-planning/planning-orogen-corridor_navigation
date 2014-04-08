/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FollowingTask.hpp"
#include <corridor_navigation/VFHFollowing.hpp>

using namespace corridor_navigation;
using namespace std;

FollowingTask::FollowingTask(std::string const& name, TaskCore::TaskState initial_state)
    : FollowingTaskBase(name, initial_state)
    , search(0)
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

    delete search;
    search = new corridor_navigation::VFHFollowing;
    search->setSearchConf(_search_conf.get());
    search->setCostConf(_cost_conf.get());
    return true;
}

void FollowingTask::updateHook()
{
    FollowingTaskBase::updateHook();

    corridor_navigation::CorridorFollowingProblem problem;
    RTT::FlowStatus status = _problem.readNewest(problem);
    if (status == RTT::NewData)
        search->setCorridor(problem.corridor, problem.desiredFinalHeading);
    else if (status == RTT::NoData)
    {
	//write empty trajectory to stop robot
	_trajectory.write(std::vector<base::Trajectory>());
        return;
    }
    
    base::samples::RigidBodyState current_pose;
    if (_pose_samples.readNewest(current_pose) == RTT::NoData)
    {
	//write empty trajectory to stop robot
	_trajectory.write(std::vector<base::Trajectory>());
        return;
    }
    try
    {
        base::Time start = base::Time::now();
        std::cerr << "starting" << std::endl;
        std::pair<base::geometry::Spline<3>, bool> result =
            search->getTrajectory(base::Pose(current_pose.position, current_pose.orientation), _search_horizon.get());
        if (result.second)
        {
	    std::cout << "Success: Horizon reached" << std::endl;
	    //write empty trajectory to stop robot
	    _trajectory.write(std::vector<base::Trajectory>());
            stop();
            return;
        }

        base::Time planning_time = (base::Time::now() - start);
        outputDebuggingTypes(planning_time);

        if (result.first.isEmpty()) {
	    //write empty trajectory to stop robot
	    _trajectory.write(std::vector<base::Trajectory>());
	    std::cout << "Error Could not compute path to horizon" << std::endl;
            return exception(NO_VIABLE_PATH);
	}
	std::vector<base::Trajectory> tr;
	tr.resize(1);
	tr[0].speed = 1;
	tr[0].spline = result.first;
        _trajectory.write(tr);

    }
    catch(std::exception const& e)
    {
        outputDebuggingTypes(base::Time());
	//write empty trajectory to stop robot
	_trajectory.write(std::vector<base::Trajectory>());
        throw;
    }

}

void FollowingTask::outputDebuggingTypes(base::Time const& planning_time)
{
    if (_debugVfhTree.connected())
    {
        const vfh_star::DebugTree *dTree = search->getDebugTree();
        if(dTree)
            _debugVfhTree.write(*dTree);
    }
    if (_debug.connected())
    {
        FollowingDebug debug;
        debug.planning_time = planning_time;
        pair<base::Vector3d, base::Vector3d> h = search->getHorizon();
        debug.horizon[0] = h.first;
        debug.horizon[1] = h.second;
        const vfh_star::DebugTree *dTree =search->getDebugTree();
        if(dTree)
        debug.tree = *dTree;
        _debug.write(debug);
    }

}

// void FollowingTask::errorHook()
// {
//     FollowingTaskBase::errorHook();
// }
void FollowingTask::stopHook()
{
    //write empty trajectory to stop robot
    _trajectory.write(std::vector<base::Trajectory>());
    FollowingTaskBase::stopHook();
}
// void FollowingTask::cleanupHook()
// {
//     FollowingTaskBase::cleanupHook();
// }

