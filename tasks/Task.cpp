#include "Task.hpp"
#include <corridor_navigation/VFHServoing.hpp>
#include <vfh_star/VFHStar.h>
#include <vfh_star/VFH.h>
#include <asguard/Transformation.hpp>

using namespace corridor_servoing;
using namespace vfh_star;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    aggr = new aggregator::StreamAligner();    
    asguard::Transformation asguardConf;
    laser2Body = asguardConf.laser2Body;
    body2Odo.setIdentity();
    
    globalHeading = 0;
}


void Task::odometry_callback(base::Time ts, const base::samples::RigidBodyState& odometry_reading)
{
    gotOdometry = true;
    body2Odo = odometry_reading;
}

void Task::scan_callback(base::Time ts, const base::samples::LaserScan& scan_reading)
{
    if(!gotOdometry)
	return;

    if(_heading.read(globalHeading) == RTT::NoData)
	return;

    bool gotNewMap = mapGenerator.addLaserScan(scan_reading, body2Odo, laser2Body);

    if(gotNewMap) {
	const TraversabilityGrid &trGridGMS(mapGenerator.getTraversabilityMap());

	//convert to envire::Grid<Traversability>
	envire::Environment env;
	envire::FrameNode gridPos;
	Transform3d tr;
	tr.setIdentity();
	tr.translation() = trGridGMS.getGridPosition();
	gridPos.setTransform(tr);
	env.attachItem(&gridPos);
	envire::Grid<Traversability> trGrid(trGridGMS.getWidth(), trGridGMS.getHeight(), trGridGMS.getGridEntrySize(), trGridGMS.getGridEntrySize());
	envire::Grid<Traversability>::ArrayType &trData = trGrid.getGridData();
	env.attachItem(&trGrid);
	trGrid.setFrameNode(&gridPos);
	for(int x = 0; x < trGridGMS.getWidth(); x++) {
	    for(int y = 0; y < trGridGMS.getHeight(); y++) {
		trData[x][y] = trGridGMS.getEntry(x, y);
	    }			
	}		    

        corridor_navigation::VFHServoing vfh(&trGrid);
        vfh.setCostConf(_cost_conf.get());
        vfh.setSearchConf(_search_conf.get());
	
	base::Time start = base::Time::now();
        std::vector<base::Waypoint> waypoints =
            vfh.getWaypoints(base::Pose(body2Odo), globalHeading, _search_horizon.get());
	base::Time end = base::Time::now();
	_trajectory.write(TreeSearch::waypointsToSpline(waypoints));
	std::cout << "vfh took " << (end-start).toMicroseconds() << std::endl; 

	//detach items, to avoid invalid free when env get's destroyed
	env.detachItem(&trGrid);
	env.detachItem(&gridPos);
	
	//write out debug output
        if (_gridDump.connected())
        {
            vfh_star::GridDump gd;
            mapGenerator.getGridDump(gd);
            _gridDump.write(gd);
        }

        if (_vfhDebug.connected())
            _vfhDebug.write(vfh.getVFHStarDebugData(waypoints));
        if (_debugVfhTree.connected())
            _debugVfhTree.write(vfh.getTree());
    }
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{    
    // setup the aggregator with the timeout value provided by the module
    aggr->setTimeout( base::Time::fromSeconds( _max_delay.value() ) );

    const double buffer_size_factor = 2.0;

    od_idx = aggr->registerStream<base::samples::RigidBodyState>(
	   boost::bind( &Task::odometry_callback, this, _1, _2 ),
	   buffer_size_factor* ceil( _max_delay.value()/_odometry_period.value() ),
	   base::Time::fromSeconds( _odometry_period.value() ) );
    
    scan_idx = aggr->registerStream<base::samples::LaserScan>(
	   boost::bind( &Task::scan_callback, this, _1, _2 ),
	   buffer_size_factor* ceil( _max_delay.value()/_scan_period.value() ),
	   base::Time::fromSeconds( _scan_period.value() ) );

    gotOdometry = false;
    
    return true;
}

// bool Task::startHook()
// {
//     return true;
// }




void Task::updateHook()
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

// void Task::errorHook()
// {
// }
// void Task::stopHook()
// {
// }
// void Task::cleanupHook()
// {
// }

