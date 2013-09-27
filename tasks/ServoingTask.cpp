#include "ServoingTask.hpp"
#include <vfh_star/VFHStar.h>
#include <vfh_star/VFH.h>
#include <envire/maps/MLSGrid.hpp>
#include <envire/Orocos.hpp>
#include <cmath>

using namespace corridor_navigation;
using namespace vfh_star;
using namespace Eigen;

ServoingTask::ServoingTask(std::string const& name)
    : ServoingTaskBase(name), bodyCenter2Odo(Affine3d::Identity()), globalHeading(0.0), 
            gotNewMap(false), justStarted(true), noTrCounter(0), failCount(0), unknownTrCounter(0), 
            unknownRetryCount(0), env(), gridPos(NULL), trGrid(NULL), vfhServoing(NULL), 
            dynamixelMin(std::numeric_limits< double >::max()), dynamixelMax(-std::numeric_limits< double >::max()), 
            dynamixelDir(0), dynamixelMaxFixed(false), dynamixelMinFixed(false), dynamixelAngle(0), 
            bodyCenter2Body(Affine3d::Identity()), markedRobotsPlace(false), aprioriMap(NULL), 
            aprioriMap2Body(Affine3d::Identity())
{    
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

    // Set correct position of grid in envire.
    Affine3d tr;
    tr.setIdentity();
    tr.translation() = trGridGMS.getGridPosition();
    gridPos->setTransform(tr);
}

void ServoingTask::updateSweepingState(Eigen::Affine3d const& transformation)
{
    Vector3d angles = transformation.rotation().eulerAngles(2,1,0);

    double current_servo_angle = angles[2]; 
    if(current_servo_angle == 0) {
        RTT::log(RTT::Warning) << "Received servo angle is 0 (not set?)" << std::endl; 
    }
    dynamixelMin = std::min(dynamixelMin, current_servo_angle);
    dynamixelMax = std::max(dynamixelMax, current_servo_angle);

    if ( !justStarted && (!dynamixelMaxFixed || !dynamixelMinFixed) ) {
        int dir;

        if (current_servo_angle > dynamixelAngle) { 
            dir = 1;
            RTT::log(RTT::Info) << "Servo is moving counter clockwise" << RTT::endlog();
        } else if ( current_servo_angle < dynamixelAngle ) {
            dir = -1;
            RTT::log(RTT::Info) << "Servo is moving clockwise" << RTT::endlog();
        } else {
            dir = 0;
        }

        if ( dynamixelDir - dir == 2 ) {
            RTT::log(RTT::Info) << "Max dynamixel angle found and set to " << dynamixelMax << RTT::endlog();
            dynamixelMaxFixed = true;
        } else if ( dynamixelDir - dir == -2 ) {
            RTT::log(RTT::Info) << "Min dynamixel angle found and set to " << dynamixelMin << RTT::endlog();
            dynamixelMinFixed = true;
        }

        if ( dir != 0 ) {
            dynamixelDir = dir;
        }
    } else 

    dynamixelAngle = current_servo_angle;

    //track sweep status
    switch (sweepStatus)
    {
        case SWEEP_DONE:
        case SWEEP_UNTRACKED:
            break;
        case WAITING_FOR_START:
            if(fabs(dynamixelMax - dynamixelAngle) < 0.05)
            {
                RTT::log(RTT::Info) << "Set sweep status to SWEEP_STARTED" << RTT::endlog(); 
                sweepStatus = SWEEP_STARTED;
            }
            break;
        case SWEEP_STARTED:
            if(fabs(dynamixelMin - dynamixelAngle) < 0.05)
            {
                RTT::log(RTT::Info) << "Set sweep status to SWEEP_DONE" << RTT::endlog(); 
                sweepStatus = SWEEP_DONE;
            }
            break;
    }
}

inline Eigen::Affine3d XFORWARD2YFORWARD(Eigen::Affine3d const& x2x)
{
    Eigen::Affine3d y2x(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d x2y(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
    return y2x * x2x * x2y;
}

inline Eigen::Affine3d YFORWARD2XFORWARD(Eigen::Affine3d const& y2y)
{
    Eigen::Affine3d y2x(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d x2y(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
    return x2y * y2y * y2x;
}

void ServoingTask::scan_samplesTransformerCallback(const base::Time& ts, const base::samples::LaserScan& scan_reading)
{
    // Hack: Setting of minDistance within MARS is not yet available.
    base::samples::LaserScan& scan_reading_non_const = const_cast<base::samples::LaserScan&>(scan_reading);
    scan_reading_non_const.minRange = _laser_scan_min_range.get() * 1000; // m 2 mm
    
    Eigen::Affine3d laser2BodyCenter;
    if(!_laser2body_center.get(ts, laser2BodyCenter, true)) {
        RTT::log(RTT::Info) << "Interpolated transformation laser2body_center not available" << RTT::endlog();
    	return;
    }

    if (_x_forward.get()) {
        laser2BodyCenter = XFORWARD2YFORWARD(laser2BodyCenter);
    }
    
    updateSweepingState(laser2BodyCenter);

    Eigen::Affine3d body_center_to_odo;
    if(!_body_center2odometry.get(ts, body_center_to_odo, true)) {
        RTT::log(RTT::Info) << "Interpolated transformation body_center2odometry not available" << RTT::endlog();
        return;
    }
    
    bodyCenter2Odo = body_center_to_odo;

    if (_x_forward.get()) {
        bodyCenter2Odo = XFORWARD2YFORWARD(bodyCenter2Odo);
    }

    if(!_body_center2body.get(ts, bodyCenter2Body, true)) {
        RTT::log(RTT::Info) << "Interpolated transformation body_center2body not available" << RTT::endlog();
        return;
	}
	
    if (_x_forward.get()) {
        bodyCenter2Body = XFORWARD2YFORWARD(bodyCenter2Body);
    }

    if(mapGenerator->moveMapIfRobotNearBoundary(bodyCenter2Odo.translation())) {
        RTT::log(RTT::Info) << "Local map has been moved, robot has reached the boundary" << RTT::endlog();    
    }
    
    //note this has to be done after moveMapIfRobotNearBoundary
    //as moveMapIfRobotNearBoundary moves the map to the robot position
    if(justStarted)
    {
        if (aprioriMap) {
            const Eigen::Affine3d aprioriMap2BodyCenter(bodyCenter2Body.inverse() * aprioriMap2Body);
            const Eigen::Affine3d apriori2LaserGrid(bodyCenter2Odo * aprioriMap2BodyCenter);
            mapGenerator->addKnowMap(aprioriMap.get(), apriori2LaserGrid);

            aprioriMap.reset(0);
            gotNewMap = true;
        } else {
            RTT::log(RTT::Warning) << "Apriori map is not available" << RTT::endlog();
        }
        RTT::log(RTT::Info) << "justStarted set to false" << RTT::endlog();
        justStarted = false;
    } 

    if ( !markedRobotsPlace && dynamixelMaxFixed && dynamixelMinFixed ) { //TODO dynamixel.. have been commented out, why?

        TreeSearchConf search_conf(_search_conf.value());

        double val = search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance;
        double front_shadow = _front_shadow_distance.get();

        if ( front_shadow <= 0.0 ) {
            double laser_height = laser2BodyCenter.translation().z() + _height_to_ground.get();
            front_shadow = laser_height / tan(-dynamixelMin);
            RTT::log(RTT::Info) << "front shadow distance from tilt: " << front_shadow << RTT::endlog();
        }
        
        // The area which the robot cannot see will be marked as traversable.
        // The y-translation is used because of this crazy asguard2rock mapping. 
        front_shadow += laser2BodyCenter.translation().y() - val / 2.0;

        // We need enough space for a point-turn
        val *= 2;
        RTT::log(RTT::Info) << "Traversable Box width and height: " << val << RTT::endlog();
        mapGenerator->markUnknownInRectangeAsTraversable(base::Pose(bodyCenter2Odo), val, val, front_shadow);
        markedRobotsPlace = true;
        RTT::log(RTT::Info) << "Robot place has been marked as traversable" << RTT::endlog();
    } else {
        RTT::log(RTT::Info) << "Robot place has NOT been marked, dynamixelMaxFixed: " << dynamixelMaxFixed << ", dynamixelMinFixed: " << dynamixelMinFixed << RTT::endlog();
    }

    // Do not add laser scans, when the robot's place is not marked
    if ( markedRobotsPlace ) {
        gotNewMap |= mapGenerator->addLaserScan(scan_reading_non_const, bodyCenter2Odo, laser2BodyCenter);
        RTT::log(RTT::Info) << "Laserscan has been added" << RTT::endlog();
    } else {
        RTT::log(RTT::Info) << "Laserscan has NOT been added, robot place not marked" << RTT::endlog();   
    }

    // Debug output of the laser2map transformation.
    base::samples::RigidBodyState laser2Map;
    if (_x_forward.get()) {
        laser2Map.setTransform(YFORWARD2XFORWARD(mapGenerator->getLaser2Map()));
    } else {
        laser2Map.setTransform(mapGenerator->getLaser2Map());
    }
    laser2Map.sourceFrame = "laser";
    laser2Map.targetFrame = "map";
    laser2Map.time = ts;
    
    _debug_laser_frame.write(laser2Map);
}

bool ServoingTask::setMap(::std::vector< ::envire::BinaryEvent > const & map, ::std::string const & mapId, ::base::samples::RigidBodyState const & mapPose)
{
    if(!justStarted)
	return false;
    
    envire::Environment env;
    env.applyEvents(map);
    
    aprioriMap = env.getItem< envire::MLSGrid >(mapId);
    if (!aprioriMap) {
        RTT::log(RTT::Warning) << "Apriori map with id " << mapId << " could not been extracted" << RTT::endlog();
        return false;
    }
    
    aprioriMap2Body = mapPose.getTransform().inverse();
    return true;
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
    RTT::log(RTT::Info) << "Boundary size has been set to " << boundarySize << RTT::endlog();
    
    mapGenerator->setBoundarySize(boundarySize);
    mapGenerator->setMaxStepSize(_search_conf.get().maxStepSize);

    failCount = _fail_count.get();
    unknownRetryCount = _unknown_retry_count.get();

    markedRobotsPlace = false;

    justStarted = true;

    return true;
}

bool ServoingTask::startHook()
{  
    if(!ServoingTaskBase::startHook())
	return false;
    
    gotNewMap = false;
    justStarted = true;
    sweepStatus = SWEEP_UNTRACKED;
    noTrCounter = 0;
    unknownTrCounter = 0;

    mapGenerator->setHeightToGround(_height_to_ground.get());
    mapGenerator->clearMap();
    mapGenerator->setGridEntriesWindowSize(_entry_window_size);

    mapGenerator->setHeightMeasureMethod(_entry_height_conf);
    copyGrid();

    vfhServoing->setNewTraversabilityGrid(trGrid);
    
    bodyCenter2Odo = Affine3d::Identity();
    
    dynamixelMin = std::numeric_limits< double >::max();
    dynamixelMax = -std::numeric_limits< double >::max();
    dynamixelMinFixed = false;
    dynamixelMaxFixed = false;
    dynamixelDir = 0;

    return true;
}




void ServoingTask::updateHook()
{
    base::samples::RigidBodyState odometry_reading;
    while( _odometry_samples.read(odometry_reading, false) == RTT::NewData ) {
        _transformer.pushDynamicTransformation( odometry_reading );	
    }

    ServoingTaskBase::updateHook();

    if (gotNewMap && markedRobotsPlace)
    {
        mapGenerator->computeNewMap();

        TreeSearchConf search_conf(_search_conf.value());
        const base::Pose curPose(bodyCenter2Odo);
        const double obstacleDist = search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance + _search_conf.get().stepDistance * 2.0;
        //mark all unknown beside the robot as obstacle, but none in front of the robot
        // Removed: unnecessary restriction of the freedom of movement
        //mapGenerator->markUnknownInRectangeAsObstacle(curPose, obstacleDist, obstacleDist, -_search_conf.get().stepDistance * 2.0);
        //RTT::log(RTT::Info) << "Marks the unknown area besides the robot as obstacles" << RTT::endlog();
    }
    
    // Output the map
    if (_gridDump.connected() && gotNewMap)
    {
        vfh_star::GridDump gd;
        mapGenerator->getGridDump(gd);

        if (_x_forward.get())
        {
            vfh_star::GridDump gd_xforward;
            int line_size = GRIDSIZE / GRIDRESOLUTION;
            for (int y = 0; y < line_size; ++y) {
                for (int x = 0; x < line_size; ++x)
                {
                    gd_xforward.height[x * line_size + (line_size - y)] = gd.height[y * line_size + x];
                    gd_xforward.max[x * line_size + (line_size - y)] = gd.max[y * line_size + x];
                    gd_xforward.interpolated[x * line_size + (line_size - y)] = gd.interpolated[y * line_size + x];
                    gd_xforward.traversability[x * line_size + (line_size - y)] = gd.traversability[y * line_size + x];
                }
            }
            gd_xforward.gridPositionX = gd.gridPositionY;
            gd_xforward.gridPositionY = -gd.gridPositionX;
            gd_xforward.gridPositionZ = -gd.gridPositionZ;
            _gridDump.write(gd_xforward);
        } else {
            _gridDump.write(gd);
        }
        RTT::log(RTT::Info) << "Output the new map" << RTT::endlog();
    }

    if(_heading.read(globalHeading) == RTT::NoData)
    {
        //write empty trajectory to stop robot
        _trajectory.write(std::vector<base::Trajectory>());
        RTT::log(RTT::Info) << "No heading available, stop robot by writing an empty trajectory" << RTT::endlog();
        return;
    } 
    
    // Plan only if required and if we have a new map
    if(_trajectory.connected() && (gotNewMap || sweepStatus == SWEEP_DONE)) {
        RTT::log(RTT::Info) << "Replanning: Trajectory connected, got a new map, sweep status done" << RTT::endlog();
        //notify the servoing that there is a new map
        vfhServoing->setNewTraversabilityGrid(trGrid);

        vfhServoing->clearDebugData();

        base::Pose frontArea(bodyCenter2Odo);
        frontArea.position += frontArea.orientation * Vector3d(0, 0.5, 0);
        vfh_star::ConsistencyStats frontArealStats = mapGenerator->checkMapConsistencyInArea(frontArea, 0.5, 0.5);

        //copy data to envire grid, TODO why?
        copyGrid();
        
        //only go onto terrain we know something about
        //or if we can not gather any more information
        if(frontArealStats.averageCertainty > 0.3 || sweepStatus == SWEEP_DONE)
        {
            RTT::log(RTT::Info) << "Create trajectory, area known (" << frontArealStats.averageCertainty << ") and sweep done" << RTT::endlog();
            RTT::log(RTT::Info) << "" << RTT::endlog(); 
            base::Time start = base::Time::now();
            std::vector<base::Trajectory> tr;
            
            //set correct Z value according to the map
            Eigen::Affine3d bodyCenter2Odo_zCorrected(bodyCenter2Odo);
            mapGenerator->getZCorrection(bodyCenter2Odo_zCorrected);
            
            VFHServoing::ServoingStatus status = vfhServoing->getTrajectories(tr, base::Pose(bodyCenter2Odo_zCorrected), globalHeading, _search_horizon.get(), bodyCenter2Body);
            base::Time end = base::Time::now();

            Eigen::Affine3d y2x(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
            if (_x_forward.get())
            {
                for (unsigned int i = 0; i < tr.size(); ++i) {
                    tr[i].spline.transform(y2x);
                }
            }
                
            _trajectory.write(tr);
            RTT::log(RTT::Info) << "vfh took " << (end-start).toMicroseconds() << RTT::endlog(); 

            if (_vfhDebug.connected()) {
	            _vfhDebug.write(vfhServoing->getVFHStarDebugData(std::vector<base::Waypoint>()));
	        }
	        
            if (_debugVfhTree.connected()) {
	            _debugVfhTree.write(vfhServoing->getTree());
	        }
               
            if(status == VFHServoing::TRAJECTORY_THROUGH_UNKNOWN)
            {
	            if(sweepStatus == SWEEP_DONE) {
	                unknownTrCounter++;
	            }
	
	            if(unknownTrCounter > unknownRetryCount)
	            {
                    RTT::log(RTT::Error) << "Quitting, trying to drive through unknown terrain" << RTT::endlog();
                    return exception(TRAJECTORY_THROUGH_UNKNOWN); // TODO exception or error states?
	            }
            } else {
	            unknownTrCounter = 0;
            }
	
            if(status == VFHServoing::NO_SOLUTION)
            {
                RTT::log(RTT::Warning) << "Could not compute trajectory towards target horizon" << RTT::endlog();

                noTrCounter++;
                if(noTrCounter > failCount) {
                    RTT::log(RTT::Error) << "Quitting, found no solution" << RTT::endlog();
                    return exception(NO_SOLUTION); // TODO exception or error states?
                }
            } else {
	            noTrCounter = 0;
            }
            
            if(sweepStatus == SWEEP_DONE) {
                RTT::log(RTT::Info) << "Set sweep status to SWEEP_UNTRACKED" << RTT::endlog();
	            sweepStatus = SWEEP_UNTRACKED;
	        }
        } else {	
            RTT::log(RTT::Info) << "Trajectory can not be computed yet, certainty: " << 
                    frontArealStats.averageCertainty << ", sweep status: " << 
                    (sweepStatus == SWEEP_DONE ? "done" : "not done") << RTT::endlog();  
            //we need to wait a full sweep
            if(sweepStatus == SWEEP_UNTRACKED)
            {
                RTT::log(RTT::Info) << "Waiting until sweep is completed" << RTT::endlog(); 
                sweepStatus = WAITING_FOR_START;
            }
            //do not write an empty trajectory here
            //if we reached this code path we allready
            //waited for a whole sweep and a valid
            //trajectory was planned.
        }
    } else {
        RTT::log(RTT::Debug) << "Trajectory port connected: " << _trajectory.connected() << 
            ", gotNewMap: " << gotNewMap << ", sweepStatus: " << sweepStatus << RTT::endlog();
    }

    gotNewMap = false;	
}

// void ServoingTask::errorHook()
// {
// }
void ServoingTask::stopHook()
{
    justStarted = true;

    //write empty trajectory to stop robot
    _trajectory.write(std::vector<base::Trajectory>());
    RTT::log(RTT::Info) << "Write empty trajectory to stop the robot" << RTT::endlog(); 
    ServoingTaskBase::stopHook();
}
// void ServoingTask::cleanupHook()
// {
// }

