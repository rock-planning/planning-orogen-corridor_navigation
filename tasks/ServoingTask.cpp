#include "ServoingTask.hpp"
#include <vfh_star/VFHStar.h>
#include <vfh_star/VFH.h>
#include <envire/maps/MLSGrid.hpp>
#include <envire/Orocos.hpp>
#include <cmath>
#include <base/Float.hpp>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

using namespace corridor_navigation;
using namespace vfh_star;
using namespace Eigen;

ServoingTask::ServoingTask(std::string const& name)
    : ServoingTaskBase(name), frontInput(_laser2body_center, this), backInput(_laser_back2body_center, this), bodyCenter2Odo(Affine3d::Identity()), globalHeading(0.0), 
            gotNewMap(false), justStarted(true), noTrCounter(0), failCount(0), unknownTrCounter(0), 
            unknownRetryCount(0), env(), gridPos(NULL), trGrid(NULL), vfhServoing(NULL),  
            markedRobotsPlace(false), aprioriMap(NULL), 
            aprioriMap2Body(Affine3d::Identity()), mLastReplan()
{   
    xForward = true;
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

ServoingTask::SweepTracker::SweepTracker()
{
    sweepStatus = TRACKER_INIT;
    noSweepLimit = 30;
    reset();
}

void ServoingTask::SweepTracker::reset()
{
    sweepMin = std::numeric_limits< double >::max();
    sweepMax = -std::numeric_limits< double >::max();
    foundMax = false;
    foundMin = false;
    lastSweepAngle = base::NaN<double>();
    noSweepCnt = 0;
}


void ServoingTask::SweepTracker::updateSweepingState(const Eigen::Affine3d& rangeData2Body)
{
    LOG_INFO_S << "updateSweepingState" << base::Time::now().toString();

    //for now we only assume a rotation around X
    Vector3d angles = rangeData2Body.rotation().eulerAngles(2,1,0);

    double currentSweepAngle = angles[1]; 
    sweepMin = std::min(sweepMin, currentSweepAngle);
    sweepMax = std::max(sweepMax, currentSweepAngle);

    //track sweep status
    switch (sweepStatus)
    {
	case TRACKER_INIT:
        {
	    if(fabs(currentSweepAngle - lastSweepAngle) < 0.01)
	    {
		noSweepCnt++;
	    }
	    else
	    {
		noSweepCnt = 0;
	    }
	    
	    //device seems not to be sweeping at all
	    if(noSweepCnt > noSweepLimit)
	    {
		sweepStatus = SWEEP_DONE;
		break;
	    }
	    
	    double distToMin = fabs(sweepMin - currentSweepAngle);
            double distToMax = fabs(sweepMax - currentSweepAngle);
	    
	    if (currentSweepAngle > lastSweepAngle && distToMin > 0.001 && distToMax > 0.001)
	    {
		foundMin = true;
	    }

	    if (currentSweepAngle < lastSweepAngle && distToMin > 0.001 && distToMax > 0.001)
	    {
		foundMax = true;
	    }
	    
	    if(foundMin && foundMax)
	      {
		sweepStatus = SWEEP_DONE;
	      }	
	    lastSweepAngle = currentSweepAngle;
        }
	    
	    break;
        case SWEEP_DONE:

            break;
        case WAITING_FOR_START:
            if(fabs(sweepMax - currentSweepAngle) < 0.3)
            {
                LOG_INFO_S << "CorridorServoing: Set sweep status to SWEEP_STARTED" << base::Time::now().toString(); 
                sweepStatus = SWEEP_STARTED;
            }
            break;
        case SWEEP_STARTED:
            if(fabs(sweepMin - currentSweepAngle) < 0.3)
            {
                LOG_INFO_S << "CorridorServoing: Set sweep status to SWEEP_DONE" << base::Time::now().toString(); 
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

void ServoingTask::scan_samples_backTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_back_sample)
{
    backInput.addLaserScan(ts, scan_samples_back_sample);
}

void ServoingTask::velodyne_scansTransformerCallback(const base::Time &ts, const ::velodyne_lidar::MultilevelLaserScan &velodyne_scans_sample1)
{
    if(!markedRobotsPlace)
        return;

    Eigen::Affine3d velodyne2bodyCenter;
    if(!_velodyne2body_center.get(ts, velodyne2bodyCenter, true)) {
        LOG_INFO_S << "CorridorServoing: Interpolated transformation laser2body_center not available" << base::Time::now().toString();
        return;
    }

    if (xForward) {
        velodyne2bodyCenter = XFORWARD2YFORWARD(velodyne2bodyCenter);
    }    
    
    Eigen::Affine3d bodyCenter2Odo;
    if(!_body_center2odometry.get(ts, bodyCenter2Odo, true)) {
        LOG_INFO_S << "CorridorServoing: Interpolated transformation body_center2odometry not available" << base::Time::now().toString();
        return;
    }

    if (xForward) {
        bodyCenter2Odo = XFORWARD2YFORWARD(bodyCenter2Odo);
    }

    if(mapGenerator->moveMapIfRobotNearBoundary(bodyCenter2Odo.translation())) {
        LOG_INFO_S << "CorridorServoing: velodyne_scansTransformerCallback: Local map has been moved, robot has reached the boundary" << base::Time::now().toString();    
    }

    
    if(!mapGenerator->getZCorrection(bodyCenter2Odo))
    {
        std::cout << "CorridorServoing: Warning, could not get Z Correction " << std::endl;
    }
    
    velodyne_lidar::MultilevelLaserScan velodyne_scans_sample;
    velodyne_lidar::ConvertHelper::filterOutliers(velodyne_scans_sample1, velodyne_scans_sample, _velodyne_maximum_angle_to_neighbor.get(), _velodyne_minimum_valid_neighbors.get());

    const Affine3d velodyne2Odometry(bodyCenter2Odo * velodyne2bodyCenter);
    
    std::vector<Vector3d> points;
    
    std::vector<std::vector<Vector3d> > scanLines;
    scanLines.reserve(32);
    
    std::vector<velodyne_lidar::MultilevelLaserScan::VerticalMultilevelScan >::const_iterator vertIt = velodyne_scans_sample.horizontal_scans.begin();
    std::vector<velodyne_lidar::MultilevelLaserScan::SingleScan>::const_iterator horIt; 

    unsigned int horCnt = 0;
    unsigned int verCnt = 0;
    unsigned invalid = 0;
    for(; vertIt != velodyne_scans_sample.horizontal_scans.end(); vertIt++)
    {
        
        AngleAxisd horizontalRotation(vertIt->horizontal_angle.getRad(), Vector3d::UnitZ());
        horCnt = 0;
        for(horIt = vertIt->vertical_scans.begin(); horIt != vertIt->vertical_scans.end(); horIt++)
        {
            //ignore invalid readings
            //ignore everything over 12 meters
            if(!velodyne_scans_sample.isRangeValid(horIt->range) || horIt->range > 12000)
            {
                invalid++;
                horCnt++;
                continue;
            }

            base::Angle verticalAngle(vertIt->vertical_start_angle + base::Angle::fromRad(vertIt->vertical_angular_resolution * horCnt));

//             if(verticalAngle > base::Angle::fromDeg(0))
//                 break;

            if(scanLines.size() <= horCnt)
            {
                scanLines.resize(horCnt + 1);
            }

            //TODO change me for X-Forward 
            AngleAxisd verticalRotation(-verticalAngle.getRad(), Vector3d::UnitX());
            
            Vector3d point = (horizontalRotation * verticalRotation ) * Vector3d(0, horIt->range/1000.0,  0);
            
            point = velodyne2Odometry * point;
            
            scanLines[horCnt].push_back(point);
            horCnt++;
        }
        verCnt ++;
    }

//     std::cout << "Got " << horCnt << " horizontal scans with " << verCnt << " points " << " invalide " << invalid << std::endl;

    
    for(std::vector<std::vector<Vector3d> >::const_iterator it = scanLines.begin();
        it != scanLines.end(); it++)
    {
        mapGenerator->addPointVector(*it);
    }
//     gotNewMap = true;
}

ServoingTask::RangeDataInput::RangeDataInput(transformer::Transformation &rangeData2Body, ServoingTask *task) : rangeData2Body(rangeData2Body), task(task)
{
    rangeData2Body.registerUpdateCallback(boost::bind(&ServoingTask::RangeDataInput::sweepTransformCallback, this, _1));
}

void ServoingTask::RangeDataInput::sweepTransformCallback(const base::Time& ts)
{
    Eigen::Affine3d body2RangeDataInput; 
    rangeData2Body.get(ts, body2RangeDataInput);
    
    tracker.updateSweepingState(body2RangeDataInput);
}

void ServoingTask::RangeDataInput::addLaserScan(const base::Time& ts, const base::samples::LaserScan& scan_reading)
{
    if(!task->markedRobotsPlace)
        return;
    
    Eigen::Affine3d rangeData2BodyCenterTR;
    if(!rangeData2Body.get(ts, rangeData2BodyCenterTR, true)) {
        LOG_INFO_S << "CorridorServoing: Interpolated transformation laser2body_center not available" << base::Time::now().toString();
        return;
    }
    /*
     * The sweepTransformCallback is only guaranteed to take place if the components tilt_scan servo_dynamixel are used.
     * In case the callBack does not happen uncomment the following line to execute it each time a laser scan is received
     */
    //sweepTransformCallback(ts);

    if (task->xForward) {
        rangeData2BodyCenterTR = XFORWARD2YFORWARD(rangeData2BodyCenterTR);
    }    
    
    Eigen::Affine3d bodyCenter2Odo;
    if(!task->_body_center2odometry.get(ts, bodyCenter2Odo, true)) {
        LOG_INFO_S << "CorridorServoing: Interpolated transformation body_center2odometry not available" << base::Time::now().toString();
        return;
    }

    if (task->xForward) {
        bodyCenter2Odo = XFORWARD2YFORWARD(bodyCenter2Odo);
    }

    if(task->mapGenerator->moveMapIfRobotNearBoundary(bodyCenter2Odo.translation())) {
        LOG_INFO_S << "CorridorServoing: RangeDataInput:: Local map has been moved, robot has reached the boundary" << base::Time::now().toString();    
    }
    
    task->gotNewMap |= task->mapGenerator->addLaserScan(scan_reading, bodyCenter2Odo, rangeData2BodyCenterTR);
    
}


void ServoingTask::scan_samplesTransformerCallback(const base::Time& ts, const base::samples::LaserScan& scan_reading)
{
    frontInput.addLaserScan(ts, scan_reading);
}

bool ServoingTask::setMap(::std::vector< ::envire::BinaryEvent > const & map, ::std::string const & mapId, ::base::samples::RigidBodyState const & mapPose)
{
    if(!justStarted)
	return false;
    
    envire::Environment env;
    env.applyEvents(map);
    
    aprioriMap = env.getItem< envire::MLSGrid >(mapId);
    if (!aprioriMap) {
        LOG_WARN_S << "CorridorServoing: Apriori map with id " << mapId << " could not been extracted" << base::Time::now().toString();
        return false;
    }
    
    aprioriMap2Body = mapPose.getTransform().inverse();
    
    if (xForward) 
    {
        aprioriMap2Body = XFORWARD2YFORWARD(aprioriMap2Body);
    }
    
    return true;
}


bool ServoingTask::attemptPlanning(double rel_heading)
{

    LOG_DEBUG_S << "AttemptPlanning starts " << base::Time::now().toString();
    bool res = false;

    bool gotConsistentMap = checkMapConsistency();
    if(gotConsistentMap)
    {
        LOG_INFO_S << "CorridorServoing: Got consistent map" << base::Time::now().toString();
        // The method doPathPlanning uses the globalHeading.
        // We change globalHeading according to the relative received     
        double currentGlobalHeading = globalHeading;
        LOG_DEBUG_S << "CorridorServoing: The initial global heading is " << globalHeading << " "  <<base::Time::now().toString();
        setHeadingFromRelative(rel_heading);
        LOG_DEBUG_S << "CorridorServoing: attempting heading " << rel_heading << " "  <<base::Time::now().toString();
        std::vector<base::Trajectory> plannedTrajectory;
        VFHServoing::ServoingStatus status = doPathPlanning(plannedTrajectory);
        res = (status == VFHServoing::TRAJECTORY_OK);
        globalHeading = currentGlobalHeading;
        LOG_DEBUG_S << "CorridorServoing: Set global heading back to original value" << currentGlobalHeading << " " << base::Time::now().toString();
        LOG_DEBUG_S << "CorridorServoing: attempt heading status: " <<  res << " " <<base::Time::now().toString();
    } // Consistent map?

    return res;
}

void ServoingTask::resetMap(void) {
    LOG_DEBUG_S<<"resetting Map";
    justStarted = true; // set initial area as traversable

    mapGenerator->setHeightToGround(_height_to_ground.get());
    mapGenerator->clearMap();
    mapGenerator->setGridEntriesWindowSize(_entry_window_size);
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
    vfhServoing->setAllowBackwardDriving(_allowBackwardsDriving.get());
    
    //maximum distance of the horizon in the map
    double boundarySize = _search_horizon.get() + _cost_conf.get().obstacleSenseRadius + _search_conf.get().stepDistance;
    
    //add a third, as a rule of thumb to avoid problems
    boundarySize *= 1.3;
    LOG_INFO_S << "CorridorServoing: Boundary size has been set to " << boundarySize << base::Time::now().toString();
    
    mapGenerator->setBoundarySize(boundarySize);
    mapGenerator->setMaxStepSize(_search_conf.get().maxStepSize);
    mapGenerator->setMaxSlope(_search_conf.get().maxSlop);

    failCount = _fail_count.get();
    unknownRetryCount = _unknown_retry_count.get();

    xForward = _x_forward.get();
    allowPlanning = _allow_planning.get();
    
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
    doPlanning = false;
    noTrCounter = 0;
    unknownTrCounter = 0;

    mapGenerator->setHeightToGround(_height_to_ground.get());
    mapGenerator->clearMap();
    mapGenerator->setGridEntriesWindowSize(_entry_window_size);

    mapGenerator->setHeightMeasureMethod(_entry_height_conf);
    copyGrid();

    vfhServoing->setNewTraversabilityGrid(trGrid);
    
    bodyCenter2Odo = Affine3d::Identity();

    frontInput.tracker.reset();
    backInput.tracker.reset();
    
    _body_center2odometry.registerUpdateCallback(boost::bind(&ServoingTask::bodyCenter2OdoCallback , this, _1));
    
    return true;
}

void ServoingTask::writeGridDump()
{
    base::Time now = base::Time::now();
    // Output the map
    if (_gridDump.connected() && ((now - lastGridDumpTime) > base::Time::fromSeconds(0.33)))
    {
        lastGridDumpTime = now;

        vfh_star::GridDump gd;
        mapGenerator->getGridDump(gd);

        if (xForward)
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
        LOG_DEBUG_S << "CorridorServoing: Output the new map" << base::Time::now().toString();
    }
}

void ServoingTask::setHeadingFromRelative(double relative_heading)
{

    Eigen::Affine3d bodyCenter2OdoRotated = bodyCenter2Odo;
    bodyCenter2OdoRotated.rotate(Eigen::AngleAxisd(relative_heading, Eigen::Vector3d::UnitZ()));
    Vector3d angles = bodyCenter2OdoRotated.rotation().eulerAngles(0,1,2);

    if(!isnan(angles[2])) {
        globalHeading = angles[2];
        LOG_DEBUG_S << "CorridorServoing: Set global heading to " << globalHeading << base::Time::now().toString();
        // Debug output of the received heading.
        base::samples::RigidBodyState rbs_heading;
        rbs_heading.setTransform(bodyCenter2OdoRotated);
        rbs_heading.sourceFrame = "robot_heading";
        rbs_heading.targetFrame = "robot";
        _debug_heading_frame.write(rbs_heading); 
    }
}

bool ServoingTask::getDriveDirection(double& driveDirection)
{
    // Request the heading, which describes a relative orientation, apply it to the current
    // orientation of the robot within the odometry frame and set globalHeading to the new z-rotation.
    double relative_heading = 0;
    double absolute_heading = 0;
    RTT::FlowStatus retDiffHeading = _heading.read(relative_heading);
    RTT::FlowStatus retAbsoluteHeading = _absolute_heading.read(absolute_heading);
    

    if((retDiffHeading == RTT::NoData) && (retAbsoluteHeading == RTT::NoData))
    {
        //write empty trajectory to stop robot
        _trajectory.write(std::vector<base::Trajectory>());
        LOG_INFO_S << "CorridorServoing: No heading available, stop robot by writing an empty trajectory" << base::Time::now().toString();
        return false;
    }
    
    if (retDiffHeading == RTT::NewData) {
        if(base::isUnset<double>(relative_heading))
        {
            globalHeading = relative_heading;
            return false;
        }

        setHeadingFromRelative(relative_heading);
        
    }
    else
    {
        if (retAbsoluteHeading == RTT::NewData) 
        {
            globalHeading = absolute_heading;
            LOG_DEBUG_S << "Received global goal heading to " << absolute_heading << base::Time::now().toString();
            LOG_DEBUG_S << "Set global heading to " << globalHeading << base::Time::now().toString();
        }
    }

    if(base::isUnset<double>(globalHeading))
    {
        return false;
    }
        
    return true;
}

bool ServoingTask::checkMapConsistency()
{
    //convert map from map generator format to envire format
    copyGrid();
    
    //notify the servoing that there is a new map
    vfhServoing->setNewTraversabilityGrid(trGrid);

    vfhServoing->clearDebugData();

    //DISABLE FRONT CHECK
    return true;
    
    base::Pose frontArea(bodyCenter2Odo);
    frontArea.position += frontArea.orientation * Vector3d(0, 1.5, 0);
    vfh_star::ConsistencyStats frontArealStats = mapGenerator->checkMapConsistencyInArea(frontArea, 0.5, 0.5);


    // Only go onto terrain we know something about or if we can not gather any more information.
    return frontArealStats.averageCertainty >= 0.3;
}

VFHServoing::ServoingStatus ServoingTask::doPathPlanning(std::vector< base::Trajectory >& result)
{
    VFHServoing::ServoingStatus ret;
    
    LOG_INFO_S << "" << base::Time::now().toString(); 
    base::Time start = base::Time::now();
    
    //set correct Z value according to the map
    Eigen::Affine3d bodyCenter2Odo_zCorrected(bodyCenter2Odo);
    mapGenerator->getZCorrection(bodyCenter2Odo_zCorrected);
    
    ret = vfhServoing->getTrajectories(result, base::Pose(bodyCenter2Odo_zCorrected), globalHeading, _search_horizon.get(), bodyCenter2Body, _min_trajectory_lenght.get());
    base::Time end = base::Time::now();
    mLastReplan = base::Time::now();

    Eigen::Affine3d y2x(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
    if (xForward)
    {
        for (unsigned int i = 0; i < result.size(); ++i) {
            result[i].spline.transform(y2x);
        }
    }
        
    LOG_INFO_S << "CorridorServoing: vfh took " << (end-start).toMicroseconds() << base::Time::now().toString(); 

    if (_vfhDebug.connected()) {
        _vfhDebug.write(vfhServoing->getVFHStarDebugData(std::vector<base::Waypoint>()));
    }
        
    if (_debugVfhTree.connected()) {
        _debugVfhTree.write(vfhServoing->getTree());
    }
    
    return ret;
}

void ServoingTask::bodyCenter2OdoCallback(const base::Time& ts)
{
//     std::cout << "bodyCenter2OdoCallback at " << ts.toMilliseconds() << std::endl; 
    if(!_body_center2odometry.get(ts, bodyCenter2Odo, false)) 
    {
        LOG_INFO_S << "CorridorServoing: Interpolated transformation body_center2odometry not available" << base::Time::now().toString();
        return;
    }
    
    if (xForward) 
    {
        bodyCenter2Odo = XFORWARD2YFORWARD(bodyCenter2Odo);
    }

    
    if(justStarted)
    {
    
        if(!_body_center2body.get(ts, bodyCenter2Body, true)) 
        {
            LOG_INFO_S << "CorridorServoing: bodyCenter2OdoCallback: Interpolated transformation body_center2body not available" << base::Time::now().toString();
            return;
        }
            
        if (xForward) 
        {
            bodyCenter2Body = XFORWARD2YFORWARD(bodyCenter2Body);
        }

        if(mapGenerator->moveMapIfRobotNearBoundary(bodyCenter2Odo.translation())) {
            LOG_INFO_S << "CorridorServoing: bodyCenter2OdoCallback: Local map has been moved, robot has reached the boundary" << base::Time::now().toString();    
        }

        Eigen::Affine3d laser2BodyCenter;
        if(!_laser2body_center.get(ts, laser2BodyCenter, true)) 
        {
            LOG_INFO_S << "CorridorServoing: bodyCenter2OdoCallback: Interpolated transformation laser2body_center not available" << base::Time::now().toString();
            return;
        }

        if (xForward) 
        {
            laser2BodyCenter = XFORWARD2YFORWARD(laser2BodyCenter);
        }

        //note this has to be done after moveMapIfRobotNearBoundary
        //as moveMapIfRobotNearBoundary moves the map to the robot position
        if (aprioriMap) {
            const Eigen::Affine3d aprioriMap2BodyCenter(bodyCenter2Body.inverse() * aprioriMap2Body);
            Eigen::Affine3d apriori2LaserGrid(bodyCenter2Odo * aprioriMap2BodyCenter);

            //turn map by 90 degrees
            if(xForward)
                //this transformation looks okward, but works... 
                apriori2LaserGrid = (Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ())) * apriori2LaserGrid.inverse()).inverse();
            
            mapGenerator->addKnowMap(aprioriMap.get(), apriori2LaserGrid);

            aprioriMap.reset(0);
            gotNewMap = true;
        } else {
            LOG_WARN_S << "CorridorServoing: Apriori map is not available" << base::Time::now().toString();
        }
//     //deregister callback
//     _body_center2odometry.registerUpdateCallback(boost::function<void (const base::Time &ts)>());

    setInitialAreaTraversable(laser2BodyCenter);
    
    LOG_INFO_S << "justStarted is now False" << base::Time::now().toString();
    justStarted = false;
    
    }
}
		
void ServoingTask::setInitialAreaTraversable(Eigen::Affine3d laser2BodyCenter)
{
	//Setting up of the initialy traversable area (better make an independent method for readability)
    //correct Z height in case we got an apriori map
    if(aprioriMap)
    {
        if(!mapGenerator->getZCorrection(bodyCenter2Odo))
        {
            std::cout << "CorridorServoing: could not get correct Z height, discarding apriori map" << std::endl;
            mapGenerator->clearMap();
        } else
        {
            //make shure the traversable rectangle is directly on the surface of the ground
            Vector3d vecToGround = bodyCenter2Odo.rotation() * Vector3d(0,0, _height_to_ground.get());
            bodyCenter2Odo.translation().z() -= vecToGround.z();
        }
    }
    TreeSearchConf search_conf(_search_conf.value());
    
    // We use the parameter front_shadow to mark the area in front of the robot which is not covered by the lidar as traversable.
    // If the front_shadow is -1.0 that means that the front_shadow length is unknown. Therefore we calculate it.
    // If other value is given in the config file it is directly used
    double front_shadow = _front_shadow_distance.get();

    if ( front_shadow == -1.0 ) {
        LOG_INFO_S << "No front shadow distance given, one is to be estimated " << base::Time::now().toString();
        double laser_height = laser2BodyCenter.translation().z() + _height_to_ground.get();
        front_shadow = frontInput.tracker.getFrontShadow(laser_height);
        LOG_INFO_S << "laser to body center x distance: " << laser2BodyCenter.translation().y() << base::Time::now().toString();
        // The y-translation is used because of this crazy asguard2rock mapping. 
        front_shadow = (fabs(laser2BodyCenter.translation().y()) + front_shadow);
        LOG_INFO_S << "Estimated front shadow distance from tilt: " << front_shadow << base::Time::now().toString();
    }
    
    // The area which the robot cannot see will be marked as traversable.
    // We define a rectangle for this initialy assumed as traversable area.
    // - Width is given by the robotWidth + obstacleSafetyDistance + stepDistance
    // - Height is given by the Width + front_shadow
    
    double travAreaWidth = (search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance)*2.0; //More than enough
    double travAreaHeight = travAreaWidth + front_shadow;

    LOG_INFO_S << "Traversable Box width and height: " << travAreaWidth << ", " << travAreaHeight << base::Time::now().toString();
    LOG_INFO_S << "Traversable Box y_offset: " << front_shadow << base::Time::now().toString();
    // What is this y_offset doing (last parameter)? It places the robot somehow higher
    //mapGenerator->markUnknownInRectangeAsTraversable(base::Pose(bodyCenter2Odo), travAreaWidth, travAreaHeight, front_shadow);
    mapGenerator->markUnknownInRectangeAsTraversable(base::Pose(bodyCenter2Odo), travAreaWidth, travAreaHeight, 0.0);

    markedRobotsPlace = true;

    LOG_INFO_S << "Robot place has been marked as traversable" << base::Time::now().toString();

}

void ServoingTask::updateHook()
{
    ServoingTaskBase::updateHook();

    //New port to allow planning or just build the local map
    //Only plan if the port do_planning is receiving True or there is no data in it
    RTT::FlowStatus retDoPlanning = _do_planning.read(doPlanning);

    if(retDoPlanning == RTT::NewData) {
        LOG_DEBUG_S<<"do planning: "<<doPlanning<<" at: "<<base::Time::now().toString();
        if(!doPlanning) {
            // Port do planning is not allowing plan. An empty trajectory is set to the trajectory follower.
            _trajectory.write(std::vector<base::Trajectory>());
            // Sets the number of attempts to -1 to indicate that we are not planning
            LOG_DEBUG_S<<"Not planning: "<<base::Time::now().toString();
            // Until we change the syskit part to use the port "is_plannning" we keep this -1 solution too
            noTrCounter = -1;
            unknownTrCounter = -1;
            // write immediately if we are not planning
        }
        else {
            // Once we give the instruction to plan again start from 0 the count of attempts
            LOG_DEBUG_S<<"Planning: "<<base::Time::now().toString();
            //noTrCounter = 0;
            //unknownTrCounter = 0;
        }
        _count_no_trajectory.write(noTrCounter);
        _count_unknown_trajectory.write(unknownTrCounter);
        _is_planning.write(doPlanning);

    }
    else if(retDoPlanning == RTT::NoData) {
        // As default do planning. Backward complatibility
        doPlanning = true;
        // write initial values for the deadend status
    }

    _count_no_trajectory.write(noTrCounter);
    _count_unknown_trajectory.write(unknownTrCounter);

    _debug_sweep_status.write((int)frontInput.tracker.getSweepStatus());

    if(justStarted)
    {
        LOG_DEBUG_S << "CorridorServoing: Waiting for inital spot to be marked as traversable" << base::Time::now().toString();
        return;
    }
    
    if (gotNewMap && markedRobotsPlace)
    {
        mapGenerator->computeNewMap();

        TreeSearchConf search_conf(_search_conf.value());
        const base::Pose curPose(bodyCenter2Odo);
        //const double obstacleDist = search_conf.robotWidth + search_conf.obstacleSafetyDistance + search_conf.stepDistance + _search_conf.get().stepDistance * 2.0;
        //mark all unknown beside the robot as obstacle, but none in front of the robot
        // Removed: unnecessary restriction of the freedom of movement
        //mapGenerator->markUnknownInRectangeAsObstacle(curPose, obstacleDist, obstacleDist, -_search_conf.get().stepDistance * 2.0);
        //LOG_INFO_S << "Marks the unknown area besides the robot as obstacles" << base::Time::now().toString();

    }

    if(gotNewMap)
    {
        writeGridDump();
    }
    gotNewMap = false;  

    //check if we got a valid heading    
    if(!getDriveDirection(globalHeading))
    {
        //write empty trajectory to stop robot
        doPlanning = false; // No goal heading -> No planning
        _is_planning.write(doPlanning);
        _trajectory.write(std::vector<base::Trajectory>());
        LOG_INFO_S << "CorridorServoing: No heading available, stop robot by writing an empty trajectory" << base::Time::now().toString();
        // As we are not planning we set these ports to -1.
        // NOTE: To make the code clearer use a port to say when its planning or not instead of a "code value" for this port
        _count_no_trajectory.write(-1);
        _count_unknown_trajectory.write(-1);
        return;
    }
     
    //do not plan if nobody listens to us
    if(!_trajectory.connected())
    {
        LOG_DEBUG_S << "CorridorServoing: Trajectory port not connected, not planning " << base::Time::now().toString();
        return;
    }


    
    //wait for the sweep to finish before we do a replan
    if(frontInput.tracker.isSweeping())// || backInput.tracker.isSweeping())
    {
        LOG_INFO_S << "CorridorServoing: Waiting for sweep to finish" << base::Time::now().toString();
        return;
    }

    bool time2plan = (((base::Time::now() - mLastReplan).toSeconds()) > _replanning_delay.get());
    LOG_INFO_S << "Passed time since last plan: " << (base::Time::now() - mLastReplan).toSeconds();

    //if we got new sensor information 
    //try to perform a replan
    if(doPlanning && allowPlanning && time2plan)
    {
        LOG_INFO_S << "CorridorServoing: Trying to plan" << base::Time::now().toString();

        bool gotConsistentMap = checkMapConsistency();

        if(gotConsistentMap)
        {

	    LOG_INFO_S << "CorridorServoing: Got consistent map" << base::Time::now().toString();

            std::vector<base::Trajectory> plannedTrajectory;
            VFHServoing::ServoingStatus status = doPathPlanning(plannedTrajectory);

            //write the trajectory. It is allways valid
            _trajectory.write(plannedTrajectory);
            
            switch(status)
            {
                case VFHServoing::TRAJECTORY_THROUGH_UNKNOWN:
                    //noTrCounter = 0;
                    noTrCounter++;
                    unknownTrCounter++;
                    frontInput.tracker.triggerSweepTracking();
                    if(unknownTrCounter > unknownRetryCount)
                    {
                        LOG_ERROR_S << "CorridorServoing: Quitting, trying to drive through unknown terrain" << base::Time::now().toString();
                        if(_allow_exception.get())
                            return exception(TRAJECTORY_THROUGH_UNKNOWN); // TODO exception or error states?
                    }
                    break;
                case VFHServoing::NO_SOLUTION:
                    unknownTrCounter = 0;
                    noTrCounter++;
                    frontInput.tracker.triggerSweepTracking();
                    if(noTrCounter > failCount) {
                        LOG_ERROR_S << "CorridorServoing: Quitting, found no solution" << base::Time::now().toString();
                        if(_allow_exception.get())
                            return exception(NO_SOLUTION); // TODO exception or error states?
                    }

                    break;
                case VFHServoing::TRAJECTORY_OK:
                    unknownTrCounter = 0;
                    noTrCounter = 0;
                    break;
            };

        }
        else
        {
	    LOG_INFO_S << "CorridorServoing: Map ist inconsisten, triggering Sweep" << base::Time::now().toString();

            //the map was inconsistent, wait a whole sweep
            //and hope that the sweep will make it consistens again
/*            frontInput.tracker.triggerSweepTracking();
            backInput.tracker.triggerSweepTracking();*/            

        }
        _count_no_trajectory.write(noTrCounter);
        _count_unknown_trajectory.write(unknownTrCounter);
    }
}

// void ServoingTask::errorHook()
// {
// }
void ServoingTask::stopHook()
{
    justStarted = true;

    //write empty trajectory to stop robot
    _trajectory.write(std::vector<base::Trajectory>());
    LOG_INFO_S << "CorridorServoing: Write empty trajectory to stop the robot" << base::Time::now().toString(); 
    ServoingTaskBase::stopHook();
}
// void ServoingTask::cleanupHook()
// {
// }

