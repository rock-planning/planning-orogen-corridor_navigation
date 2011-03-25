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
    
    envire::Grid<Traversability>::ArrayType &trData = trGrid->getGridData();
    for(int x = 0; x < trGridGMS.getWidth(); x++) {
	for(int y = 0; y < trGridGMS.getHeight(); y++) {
	    trData[x][y] = TRAVERSABLE;
	}			
    }		    	
    
    vfhServoing = new corridor_navigation::VFHServoing(trGrid);
    
    gotNewMap = false;
}

ServoingTask::~ServoingTask() {}

void ServoingTask::scan_samplesTransformerCallback(const base::Time& ts, const base::samples::LaserScan& scan_reading)
{
    if(_heading.read(globalHeading) == RTT::NoData)
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
    

    asguard::Transformation asguardConf;
    asguard::Transformation tf;
    
    base::samples::RigidBodyState ls2Body;
    ls2Body.sourceFrame = "laser";
    ls2Body.targetFrame = "body";
    ls2Body.setTransform(tf.laser2Body);
    transformer.pushStaticTransformation(ls2Body);    

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
    
    ServoingTaskBase::updateHook();
    
    //if we got a new map replan
    if(gotNewMap) {
	const TraversabilityGrid &trGridGMS(mapGenerator.getTraversabilityMap());

	vfhServoing->clearDebugData();
	
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
	
	
	base::Time start = base::Time::now();
	std::vector<base::Waypoint> waypoints;
// 	try {
	    waypoints = vfhServoing->getWaypoints(base::Pose(body2Odo), globalHeading, _search_horizon.get());
/*	} catch(...)
	{
	    std::cerr << "Unable to get Trajectory" << std::endl;
	}*/
	base::Time end = base::Time::now();
	_trajectory.write(TreeSearch::waypointsToSpline(waypoints));
	std::cout << "vfh took " << (end-start).toMicroseconds() << std::endl; 
	
	//write out debug output
        if (_gridDump.connected())
        {
            vfh_star::GridDump gd;
            mapGenerator.getGridDump(gd);
            _gridDump.write(gd);
        }

        if (_vfhDebug.connected())
            _vfhDebug.write(vfhServoing->getVFHStarDebugData(waypoints));
        if (_debugVfhTree.connected())
            _debugVfhTree.write(vfhServoing->getTree());
	
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

