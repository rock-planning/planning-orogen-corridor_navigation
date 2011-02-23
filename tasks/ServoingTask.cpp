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
    aggr = new aggregator::StreamAligner();    
    asguard::Transformation asguardConf;
    laser2Body = asguardConf.laser2Body;
    body2Odo.setIdentity();
    
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
}


void ServoingTask::odometry_callback(base::Time ts, const base::samples::RigidBodyState& odometry_reading)
{
//     std::cout << "Body2Odo " << odometry_reading.time.toMilliseconds() << " " << odometry_reading.position.transpose() << std::endl;
    gotOdometry = true;
    body2Odo = odometry_reading;
}

void ServoingTask::scan_callback(base::Time ts, const base::samples::LaserScan& scan_reading)
{
    if(!gotOdometry)
	return;

    if(_heading.read(globalHeading) == RTT::NoData)
	return;

//     std::cout << "Scan Time Callback " << ts.toMilliseconds() << " " << body2Odo.translation().transpose() << std::endl;
    bool gotNewMap = mapGenerator.addLaserScan(scan_reading, body2Odo, laser2Body);

    
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
    }
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
    
    // setup the aggregator with the timeout value provided by the module
    aggr->setTimeout( base::Time::fromSeconds( _max_delay.value() ) );

    const double buffer_size_factor = 2.0;

    od_idx = aggr->registerStream<base::samples::RigidBodyState>(
	   boost::bind( &ServoingTask::odometry_callback, this, _1, _2 ),
	   buffer_size_factor* ceil( _max_delay.value()/_odometry_period.value() ),
	   base::Time::fromSeconds( _odometry_period.value() ) );
    
    scan_idx = aggr->registerStream<base::samples::LaserScan>(
	   boost::bind( &ServoingTask::scan_callback, this, _1, _2 ),
	   buffer_size_factor* ceil( _max_delay.value()/_scan_period.value() ),
	   base::Time::fromSeconds( _scan_period.value() ) );

    gotOdometry = false;
    
    return true;
}

// bool ServoingTask::startHook()
// {
//     return true;
// }




void ServoingTask::updateHook()
{
    base::samples::LaserScan scan_reading;
    while( _scan_samples.read(scan_reading) == RTT::NewData )
    {
	aggr->push( scan_idx, scan_reading.time, scan_reading );	
    }
    
    base::samples::RigidBodyState odometry_reading;
    while( _odometry_samples.read(odometry_reading) == RTT::NewData )
    {
	aggr->push( od_idx, odometry_reading.time, odometry_reading );	
    }

    // then call the streams in the relevant order
    while( aggr->step() );    
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

