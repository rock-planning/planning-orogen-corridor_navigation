#include "ServoingTask.hpp"
#include <vfh_star/VFHStar.h>
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
    
    mapGenerator = new vfh_star::TraversabilityMapGenerator();

    const TraversabilityGrid &trGridGMS(mapGenerator->getTraversabilityMap());

    trGrid = new envire::Grid<Traversability>(trGridGMS.getWidth(), trGridGMS.getHeight(), trGridGMS.getGridEntrySize(), trGridGMS.getGridEntrySize());
    env.attachItem(trGrid);
    trGrid->setFrameNode(gridPos);

    copyGrid();

    vfhServoing = new corridor_navigation::VFHServoing();
    vfhServoing->setNewTraversabilityGrid(trGrid);
    gotNewMap = false;
    
    body2Odo = Affine3d::Identity();
    
    dynamixelMin = std::numeric_limits< double >::max();
    dynamixelMax = -std::numeric_limits< double >::max();
}

ServoingTask::~ServoingTask() {}

void ServoingTask::copyGrid() 
{
    const TraversabilityGrid &trGridGMS(mapGenerator->getTraversabilityMap());
    envire::Grid<Traversability>::ArrayType &trData = trGrid->getGridData();
    for(int x = 0; x < trGridGMS.getWidth(); x++) 
    {
	for(int y = 0; y < trGridGMS.getHeight(); y++) 
	{
	    trData[x][y] = trGridGMS.getEntry(x, y);
	}
    }	

    //set correct position of grid in envire
    Affine3d tr;
    tr.setIdentity();
    tr.translation() = trGridGMS.getGridPosition();
    gridPos->setTransform(tr);
}

void ServoingTask::updateSweepingState(Eigen::Affine3d const& transformation)
{
    Vector3d angles = transformation.rotation().eulerAngles(2,1,0);
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
            {
                std::cout << "Sweep started" << std::endl; 
                sweepStatus = SWEEP_STARTED;
            }
            break;
        case SWEEP_STARTED:
            if(fabs(dynamixelMin - dynamixelAngle) < 0.05)
            {
                std::cout << "Sweep done" << std::endl; 
                sweepStatus = SWEEP_DONE;
            }
            break;
    }
}

void ServoingTask::scan_samplesTransformerCallback(const base::Time& ts, const base::samples::LaserScan& scan_reading)
{
    Eigen::Affine3d laser2Body;
    if(!_laser2body.get(ts, laser2Body, true))
	return;

    updateSweepingState(laser2Body);
    
    if(!_body2odometry.get(ts, body2Odo, true))
	return;

    if(justStarted)
    {
        TreeSearchConf search_conf(_search_conf.value());
        double val = search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance;
        //TODO calulate distance to laser beam inpackt based on laser angle
        mapGenerator->markUnknownInRectangeAsTraversable(base::Pose(body2Odo), val, val, 0.3);
        justStarted = false;
    } 

    gotNewMap |= mapGenerator->addLaserScan(scan_reading, body2Odo, laser2Body);
    
    base::samples::RigidBodyState laser2Map;
    laser2Map.setTransform(mapGenerator->getLaser2Map());
    laser2Map.sourceFrame = "laser";
    laser2Map.targetFrame = "map";
    laser2Map.time = ts;
    
    _debug_laser_frame.write(laser2Map);
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ServoingTask.hpp for more detailed
// documentation about them.

bool ServoingTask::configureHook()
{    
    if (!ServoingTaskBase::configureHook())
        return false;

    vfhServoing->setCostConf(_cost_conf.get());
    vfhServoing->setSearchConf(_search_conf.get());
    
    //maximum distance of the horizon in the map
    double boundarySize = _search_horizon.get() + _cost_conf.get().obstacleSenseRadius + _search_conf.get().stepDistance;
    
    //add a third, as a rule of thumb to avoid problems
    boundarySize *= 1.3;
    
    mapGenerator->setBoundarySize(boundarySize);
    mapGenerator->setMaxStepSize(_search_conf.get().maxStepSize);

    return true;
}

bool ServoingTask::startHook()
{  
    if(!ServoingTaskBase::startHook())
	return false;
    
    gotNewMap = false;
    justStarted = true;
    sweepStatus = SWEEP_UNTRACKED;

    mapGenerator->clearMap();
    copyGrid();

    vfhServoing->setNewTraversabilityGrid(trGrid);
    
    body2Odo = Affine3d::Identity();
    
    dynamixelMin = std::numeric_limits< double >::max();
    dynamixelMax = -std::numeric_limits< double >::max();
    return true;
}




void ServoingTask::updateHook()
{
    base::samples::RigidBodyState odometry_reading;
    while( _odometry_samples.read(odometry_reading, false) == RTT::NewData )
	_transformer.pushDynamicTransformation( odometry_reading );	

    ServoingTaskBase::updateHook();

    if (gotNewMap)
    {
	mapGenerator->computeNewMap();

	TreeSearchConf search_conf(_search_conf.value());
	const base::Pose curPose(body2Odo);
	const double obstacleDist = search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance + _search_conf.get().stepDistance * 2.0;
	//mark all unknown beside the robot as obstacle, but none in front of the robot
	mapGenerator->markUnknownInRectangeAsObstacle(curPose, obstacleDist, obstacleDist, -_search_conf.get().stepDistance * 2.0);
    }
    
    // Output the map
    if (_gridDump.connected() && gotNewMap)
    {
        vfh_star::GridDump gd;
        mapGenerator->getGridDump(gd);
        _gridDump.write(gd);
    }

    if(_heading.readNewest(globalHeading) == RTT::NoData)
        return;
    
    // Plan only if required and if we have a new map
    if(_trajectory.connected() || (gotNewMap || sweepStatus == SWEEP_DONE)) {
	//notify the servoing that there is a new map
	vfhServoing->setNewTraversabilityGrid(trGrid);
	
	const base::Pose curPose(body2Odo);
	
	vfhServoing->clearDebugData();
	
	base::Pose frontArea(curPose);
	frontArea.position += curPose.orientation * Vector3d(0, 0.5, 0);
	vfh_star::ConsistencyStats frontArealStats = mapGenerator->checkMapConsistencyInArea(frontArea, 0.5, 0.5);
	
	//copy data to envire grid
	copyGrid();
	
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
           
           if(waypoints.empty())
           {
               std::cout << "Could not compute trajectory towards target horizon" << std::endl;
               return exception();
           }
	} else {	    
	    //we need to wait a full sweep
	    if(sweepStatus == SWEEP_UNTRACKED)
	    {
		std::cout << "Waiting until sweep is completed" << std::endl; 
		sweepStatus = WAITING_FOR_START;
	    }

	    std::vector<base::Waypoint> waypoints;	    
	    _trajectory.write(TreeSearch::waypointsToSpline(waypoints));
	}
    }

    gotNewMap = false;	
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

