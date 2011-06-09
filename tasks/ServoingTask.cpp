#include "ServoingTask.hpp"
#include <vfh_star/VFHStar.h>
#include <asguard/Transformation.hpp>
#include <vfh_star/VFH.h>

using namespace corridor_navigation;
using namespace vfh_star;
using namespace Eigen;

ServoingTask::ServoingTask(std::string const& name)
    : ServoingTaskBase(name)
{
    globalHeading = 0;
    
    gridPos = new envire::FrameNode();
    env.attachItem(gridPos);
    
    const TraversabilityGrid &trGridGMS(mapGenerator.getTraversabilityMap());

    trGrid = new envire::Grid<Traversability>(trGridGMS.getWidth(), trGridGMS.getHeight(), trGridGMS.getGridEntrySize(), trGridGMS.getGridEntrySize());
    env.attachItem(trGrid);
    trGrid->setFrameNode(gridPos);
    
    envire::Grid<Traversability>::ArrayType &gridData = trGrid->getGridData();

    for(int x = 0;x < trGrid->getWidth(); x++)
    {
	for(int y = 0;y < trGrid->getHeight(); y++)
	{
	    gridData[x][y] = UNCLASSIFIED;
	}
    }
    
    vfhServoing = new corridor_navigation::VFHServoing();
    vfhServoing->setNewTraversabilityGrid(trGrid);
    gotNewMap = false;
    
    body2Odo = Transform3d::Identity();
    
    dynamixelMin = std::numeric_limits< double >::max();
    dynamixelMax = -std::numeric_limits< double >::max();
}

ServoingTask::~ServoingTask() {}

void ServoingTask::scan_samplesTransformerCallback(const base::Time& ts, const base::samples::LaserScan& scan_reading)
{
    if(_heading.readNewest(globalHeading) == RTT::NoData)
    {
	std::cout << "No Heading" << std::endl;
	return;
    }
    
    Eigen::Transform3d laser2Body;
    
    if(!_laser2body.get(ts, laser2Body, true))
    {
	std::cout << "No laser2Body" << std::endl;
	return;
    }
    
    if(!_body2odometry.get(ts, body2Odo, true))
    {
	std::cout << "No body2Odo" << std::endl;
	return;
    }

    gotNewMap |= mapGenerator.addLaserScan(scan_reading, body2Odo, laser2Body);
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ServoingTask.hpp for more detailed
// documentation about them.

bool ServoingTask::configureHook()
{    
    vfhServoing->setCostConf(_cost_conf.get());
    vfhServoing->setSearchConf(_search_conf.get());
    
    //maximum distance of the horizon in the map
    double boundarySize = _search_horizon.get() + _cost_conf.get().obstacleSenseRadius + _search_conf.get().stepDistance;
    
    //add a third, as a rule of thumb to avoid problems
    boundarySize *= 1.3;
    
    mapGenerator.setBoundarySize(boundarySize);
    mapGenerator.setMaxStepSize(_search_conf.get().maxStepSize);

    asguard::Transformation asguardConf;
    asguard::Transformation tf;

    base::samples::RigidBodyState lowerDyn2Head;
    lowerDyn2Head.sourceFrame = "lower_dynamixel";
    lowerDyn2Head.targetFrame = "head";
    lowerDyn2Head.setTransform(tf.lowerDynamixel2Head);
    transformer.pushStaticTransformation(lowerDyn2Head);

    base::samples::RigidBodyState tiltHead2UpperDyn;
    tiltHead2UpperDyn.sourceFrame = "tilt_head";
    tiltHead2UpperDyn.targetFrame = "upper_dynamixel";
    tiltHead2UpperDyn.setTransform(tf.tiltHead2UpperDynamixel);
    transformer.pushStaticTransformation(tiltHead2UpperDyn);
    
    base::samples::RigidBodyState laser2TiltHead;
    laser2TiltHead.sourceFrame = "laser";
    laser2TiltHead.targetFrame = "tilt_head";
    laser2TiltHead.setTransform(tf.laser2TiltHead);
    transformer.pushStaticTransformation(laser2TiltHead);
    
    base::samples::RigidBodyState head2Body;
    head2Body.sourceFrame = "head";
    head2Body.targetFrame = "body";
    head2Body.setTransform(tf.head2Body);
    transformer.pushStaticTransformation(head2Body);    

    //static case
    /*    base::samples::RigidBodyState lowerDyn2UpperDyn;
    lowerDyn2UpperDyn.sourceFrame = "lower_dynamixel";
    lowerDyn2UpperDyn.targetFrame = "upper_dynamixel";
    lowerDyn2UpperDyn.setTransform(Transform3d(Transform3d::Identity()));
    transformer.pushStaticTransformation(lowerDyn2UpperDyn);    */

    gotNewMap = false;
    afterConfigure = true;
    sweepStatus = SWEEP_UNTRACKED;
    
    return ServoingTaskBase::configureHook();
}

// bool ServoingTask::startHook()
// {
//     return true;
// }




void ServoingTask::updateHook()
{
    base::samples::RigidBodyState odometry_reading;
    while( _odometry_samples.read(odometry_reading, false) == RTT::NewData )
    {
	transformer.pushDynamicTransformation( odometry_reading );	
    }

    base::samples::RigidBodyState dynamixel_reading;
    while( _dynamixel_samples.read(dynamixel_reading, false) == RTT::NewData )
    {
	transformer.pushDynamicTransformation( dynamixel_reading );	

	Vector3d angles = dynamixel_reading.orientation.toRotationMatrix().eulerAngles(2,1,0);
	dynamixelMin = std::min(dynamixelMin, angles[2]);
	dynamixelMax = std::max(dynamixelMax, angles[2]);

	dynamixelAngle = angles[2];

	//track sweep status
	switch (sweepStatus)
	{
	    case SWEEP_DONE:
	    case SWEEP_UNTRACKED:
		break;
	    case WAITING_FOR_START:
		if(fabs(dynamixelMax - dynamixelAngle) < 0.05)
		    sweepStatus = SWEEP_STARTED;
		break;
	    case SWEEP_STARTED:
		if(fabs(dynamixelMin - dynamixelAngle) < 0.05)
		    sweepStatus = SWEEP_DONE;
		break;
	}
    }
    
    ServoingTaskBase::updateHook();
    
    //if we got a new map replan
    if(gotNewMap) {
	//notify the servoing that there is a new map
	vfhServoing->setNewTraversabilityGrid(trGrid);
	
	TreeSearchConf search_conf(_search_conf.get());
	
	const base::Pose curPose(body2Odo);
	if(afterConfigure)
	{
	    double val = search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance;
	    //TODO calulate distance to laser beam inpackt based on laser angle
	    mapGenerator.markUnknownInRectangeAsTraversable(curPose, val, val, 0.3);
	    afterConfigure = false;
	} 

	mapGenerator.computeNewMap();
	const double obstacleDist = search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance + _search_conf.get().stepDistance * 2.0;
	//mark all unknown beside the robot as obstacle, but none in front of the robot
	mapGenerator.markUnknownInRectangeAsObstacle(curPose, obstacleDist, obstacleDist, -_search_conf.get().stepDistance * 2.0);
	
	const TraversabilityGrid &trGridGMS(mapGenerator.getTraversabilityMap());

	vfhServoing->clearDebugData();
	
	base::Pose frontArea(curPose);
	frontArea.position += curPose.orientation * Vector3d(0, 0.5, 0);
	
	vfh_star::ConsistencyStats frontArealStats = mapGenerator.checkMapConsistencyInArea(frontArea, 0.5, 0.5);
	
	//set correct position of grid in envire
	Transform3d tr;
	tr.setIdentity();
	tr.translation() = trGridGMS.getGridPosition();
	gridPos->setTransform(tr);
	
	//copy data to envire grid
	envire::Grid<Traversability>::ArrayType &trData = trGrid->getGridData();
	for(int x = 0; x < trGridGMS.getWidth(); x++) {
	    for(int y = 0; y < trGridGMS.getHeight(); y++) {
		trData[x][y] = trGridGMS.getEntry(x, y);
	    }
	}
	
	
	//only go onto terrain we know something about
	//or if we can not gather any more information
	if(frontArealStats.averageCertainty > 0.3 || sweepStatus == SWEEP_DONE)
	{
	    if(sweepStatus == SWEEP_DONE)
		sweepStatus = SWEEP_UNTRACKED;

	    base::Time start = base::Time::now();

	    std::vector<base::Waypoint> waypoints;	    
	    waypoints = vfhServoing->getWaypoints(curPose, globalHeading, _search_horizon.get());
	    
	    base::Time end = base::Time::now();

	    _trajectory.write(TreeSearch::waypointsToSpline(waypoints));
	    std::cout << "vfh took " << (end-start).toMicroseconds() << std::endl; 

	    if (_vfhDebug.connected())
		_vfhDebug.write(vfhServoing->getVFHStarDebugData(waypoints));
	    if (_debugVfhTree.connected())
		_debugVfhTree.write(vfhServoing->getTree());
	} else {	    
	    //we need to wait a full sweep
	    if(sweepStatus == SWEEP_UNTRACKED)
		sweepStatus = WAITING_FOR_START;
	}
	
	
	//write out debug output
        if (_gridDump.connected())
        {
            vfh_star::GridDump gd;
            mapGenerator.getGridDump(gd);
            _gridDump.write(gd);
        }

	
	gotNewMap = false;
    }
}

// void ServoingTask::errorHook()
// {
// }
// void ServoingTask::stopHook()
// {
// }
// void ServoingTask::cleanupHook()
// {
// }

