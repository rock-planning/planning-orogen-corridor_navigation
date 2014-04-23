#ifndef CORRIDOR_NAVIGATION_TASK_TASK_HPP
#define CORRIDOR_NAVIGATION_TASK_TASK_HPP

#include "corridor_navigation/ServoingTaskBase.hpp"
#include <corridor_navigation/VFHServoing.hpp>
#include <Eigen/Core>
#include <envire/maps/TraversabilityGrid.hpp>
#include <trajectory_follower/TrajectoryTargetCalculator.hpp>
#include <tilt_scan/tilt_scanTypes.hpp>

namespace corridor_navigation {
    
    class SweepTracker
    {
        std::map<std::string, tilt_scan::SweepStatus> lastStates;
        std::map<std::string, bool> trackStates;
        bool isTracking;
        
    public:
        SweepTracker() : isTracking(false)
        {
        }
        
        void updateTracker(tilt_scan::SweepStatus &curState)
        {
            if(isTracking)
            {
                std::map<std::string, tilt_scan::SweepStatus>::iterator it = lastStates.find(curState.sourceName);
                if(it == lastStates.end())
                {
                    lastStates[curState.sourceName] = curState;
                    trackStates[curState.sourceName] = true;
                    return;
                }
                if(curState.isNextSweep(it->second))
                {
                    trackStates[curState.sourceName] = false;
                    //check if all states are done
                    for(std::map<std::string, bool>::const_iterator it = trackStates.begin(); it != trackStates.end(); it++)
                    {
                        if(it->second == true)
                            return;
                    }
                    isTracking = false;
                }
            }
            else
            {
                lastStates[curState.sourceName] = curState;
            }
        }
        
        void triggerSweepTracking()
        {
            if(lastStates.empty())
                return;
            
            isTracking = true;
            for(std::map<std::string, tilt_scan::SweepStatus>::iterator it = lastStates.begin(); it != lastStates.end(); it++)
            {
                trackStates[it->first] = true;
            }
        };
        
        bool areSweepsDone() const
        {
            return !isTracking;
        };

        void reset()
        {
            isTracking = false;
            trackStates.clear();
            lastStates.clear();
        }
    };
    
  
    
    class ServoingTask : public ServoingTaskBase
    {
	friend class ServoingTaskBase;
    protected:
        SweepTracker frontTracker;
        SweepTracker backTracker;
        
        void transformationCallback(const base::Time &ts, transformer::Transformation &tr, Eigen::Affine3d &value, bool &gotIt);
        
        void bodyCenter2MapCallback(const base::Time &ts);
        void bodyCenter2TrajectoryCallback(const base::Time &ts);
        void bodyCenter2GlobalTrajectoryCallback(const base::Time &ts);
        
        ///Last transformation from body to trajectory coorinate frame
	Eigen::Affine3d bodyCenter2Trajectory;
        ///Last transformation from body to map coorinate frame
        Eigen::Affine3d bodyCenter2Map;

        ///Last transformation from body to global trajectorie coorinate frame
        Eigen::Affine3d bodyCenter2GlobalTrajectorie;

        bool gotBodyCenter2Trajectory;
        bool gotBodyCenter2GlobalTrajectory;
        bool gotBodyCenter2Map;
        
	bool gotNewMap;

	///Current consecutive planning tries that failed.
	int noTrCounter;
	///Maximum number of planning tries that may fail.
	int failCount;

	/**How often the trajectorie went through unknow terrain
	* and was cut down
	*/
	int unknownTrCounter;
	///Max number of sweep to wait in case of trajectory through unknown terrain
	int unknownRetryCount;
        
        bool didConsistencySweep;
        double minDriveProbability;
	
	envire::Environment env;

	envire::FrameNode *gridPos;
	envire::TraversabilityGrid *trGrid;
	
	corridor_navigation::VFHServoing vfhServoing;
        
        std::vector<base::Trajectory> trajectories;
        trajectory_follower::TrajectoryTargetCalculator trTargetCalculator;
	base::Time lastSuccessfullPlanning;
	        
        bool hasHeading_map;
        base::Angle heading_map;
        
        bool getDriveDirection(base::Angle& result);
        bool getMap();
        bool getGlobalTrajectory();
        
        bool doPathPlanning();
        
        void consistencyCallback(size_t x, size_t y, double &sum, int &cnt);
        bool isMapConsistent();
        
        SweepTracker sweepTracker;
        
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ServoingTask(std::string const& name = "corridor_navigation::ServoingTask");
        ~ServoingTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called. See README.txt for different
         * triggering options.
         *
         * The warning(), error() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeWarning, RunTimeError and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recovered()
         * allows you to go back into the Running state.  In the second case,
         * the errorHook() will be called instead of updateHook() and in the
         * third case the component is stopped and resetError() needs to be
         * called before starting it again.
         *
         * The \a updated_ports argument is the set of ports that have triggered
         * this call. If the trigger is caused by something different (for
         * instance, a periodic update), then this set is empty.
         */
        void updateHook();
        

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
    };
}

#endif

