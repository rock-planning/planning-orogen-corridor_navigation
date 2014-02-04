#ifndef CORRIDOR_NAVIGATION_TASK_TASK_HPP
#define CORRIDOR_NAVIGATION_TASK_TASK_HPP

#include "corridor_navigation/ServoingTaskBase.hpp"
#include <corridor_navigation/VFHServoing.hpp>
#include <vfh_star/TraversabilityMapGenerator.h>
#include <Eigen/Core>
#include <envire/maps/MLSGrid.hpp>

namespace corridor_navigation {
    
    enum SweepStatus {
	TRACKER_INIT,
        WAITING_FOR_START,
        SWEEP_STARTED,
        SWEEP_DONE,
    };
    
    class ServoingTask : public ServoingTaskBase
    {
	friend class ServoingTaskBase;
    protected:
        
        class SweepTracker
        {
            double sweepMin;
            double sweepMax;
            
            double lastSweepAngle;
            
            int noSweepCnt;
            int noSweepLimit;
            
            bool foundMin;
            bool foundMax;
            
            SweepStatus sweepStatus;
        public:
            SweepTracker();
            void updateSweepingState(Eigen::Affine3d const& rangeDataInput2Body);
            
            void reset();
            
            double getMinAngle() const 
            {
                return sweepMin;
            }

            double getMaxAngle() const 
            {
                return sweepMax;
            }

            SweepStatus getSweepStatus() const
            {
                return sweepStatus;
            };
            
            bool isSweeping()
            {
	        return (sweepStatus != SWEEP_DONE);
            }
            
            void triggerSweepTracking()
            {
                if(sweepStatus == SWEEP_DONE)
                    sweepStatus = WAITING_FOR_START;
            };
            
            bool isSweepDone() const
            {
	        return (sweepStatus == SWEEP_DONE);
            };
            
            
            bool initDone() const
            {
	        return (sweepStatus != TRACKER_INIT);
            }

        };
        
        
        class RangeDataInput {
        public:
            RangeDataInput(transformer::Transformation &rangeData2Body, ServoingTask *task);
            void addLaserScan(const base::Time& ts, const base::samples::LaserScan& scan_reading); 
            SweepTracker tracker;
        private:
            transformer::Transformation &rangeData2Body;
            void sweepTransformCallback(const base::Time &ts);
            ServoingTask *task;
        };
        
        RangeDataInput frontInput;
        RangeDataInput backInput;
        
	/** Handler for the setMap operation
        */
        virtual bool setMap(::std::vector< ::envire::BinaryEvent > const & map, ::std::string const & mapId, ::base::samples::RigidBodyState const & mapPose);
	
	/** instance of the TraversabilityMapGenerator, which generates a traversability map from
	 * Odometry and laserscans */
	vfh_star::TraversabilityMapGenerator *mapGenerator;

        virtual void scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample);

        virtual void scan_samples_backTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_back_sample);

        void bodyCenter2OdoCallback(const base::Time &ts);
        
	/**
	 * Copies the data from the map generator to trGrid 
	 **/
	void copyGrid();
	
	///Last transformation from body to odometry
	Eigen::Affine3d bodyCenter2Odo;
        Eigen::Affine3d bodyCenter2Body;
	
	/** heading, where the robot should drive */
	double globalHeading;
	
	bool gotNewMap;
	bool doPlanning;
	bool allowPlanning;
	// True just after the startHook, and until we get a proper pose update
	bool justStarted;

        bool xForward;
        
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
	
	envire::Environment env;

	envire::FrameNode *gridPos;
	envire::Grid<vfh_star::Traversability> *trGrid;
	
	corridor_navigation::VFHServoing *vfhServoing;
    
	bool markedRobotsPlace; //!< The robots place plus the region in front of it is marked as traversable.
	
	/**
	 * Apriori map of the environment
	 * */
	envire::MLSGrid::Ptr aprioriMap; 
	
	/**
	 * Transformation from apriori map to body frame
	 * */
	Eigen::Affine3d aprioriMap2Body;
	
	base::Time mLastReplan;
	
        /**
         * Writes the current grid map onto a port
         * */
        void writeGridDump();
        
        bool getDriveDirection(double &driveDirection);
        
        VFHServoing::ServoingStatus doPathPlanning(std::vector< base::Trajectory >& result);
        
        bool checkMapConsistency();
        base::Time lastGridDumpTime;
        
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

