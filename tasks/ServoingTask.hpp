#ifndef CORRIDOR_NAVIGATION_TASK_TASK_HPP
#define CORRIDOR_NAVIGATION_TASK_TASK_HPP

#include "corridor_navigation/ServoingTaskBase.hpp"
#include <corridor_navigation/VFHServoing.hpp>
#include <vfh_star/TraversabilityMapGenerator.h>
#include <Eigen/Core>

namespace corridor_navigation {
    
    enum SweepStatus {
	WAITING_FOR_START,
	SWEEP_STARTED,
	SWEEP_DONE,
	SWEEP_UNTRACKED,
    };
    
    class ServoingTask : public ServoingTaskBase
    {
	friend class ServoingTaskBase;
    protected:
	/** instance of the TraversabilityMapGenerator, which generates a traversability map from
	 * Odometry and laserscans */
	vfh_star::TraversabilityMapGenerator *mapGenerator;
	
	/** callback to receive laser scanner readings*/
	void scan_callback( base::Time ts, const base::samples::LaserScan& scan_reading );

	virtual void scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample);

	/**
	 * Copies the data from the map generator to trGrid 
	 **/
	void copyGrid();
	
	///Last transformation from body to odometry
	Eigen::Affine3d body2Odo;
	
	/** heading, where the robot should drive */
	double globalHeading;
	
	bool gotNewMap;
	
        // True just after the startHook, and until we get a proper pose update
	bool justStarted;

	envire::Environment env;

	envire::FrameNode *gridPos;
	envire::Grid<vfh_star::Traversability> *trGrid;
	
	corridor_navigation::VFHServoing *vfhServoing;
    
	double dynamixelMin;
	double dynamixelMax;
	SweepStatus sweepStatus;
	
	double dynamixelAngle;
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
        // void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
    };
}

#endif

