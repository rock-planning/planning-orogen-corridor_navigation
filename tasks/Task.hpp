#ifndef CORRIDOR_SERVOING_TASK_TASK_HPP
#define CORRIDOR_SERVOING_TASK_TASK_HPP

#include "corridor_servoing/TaskBase.hpp"
#include <StreamAligner.hpp>
#include <vfh_star/TraversabilityMapGenerator.h>
#include <Eigen/Core>

namespace RTT
{
    class NonPeriodicActivity;
}


namespace corridor_servoing {
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	/** instance of the TraversabilityMapGenerator, which generates a traversability map from
	 * Odometry and laserscans */
	vfh_star::TraversabilityMapGenerator mapGenerator;
	
	/** index for the odometry aggregator channel */
	int od_idx;
    
	/** index for the scan match aggregator channel */
	int scan_idx;
	
	/** instance of the aggregator which aligns the readings in time */
	aggregator::StreamAligner *aggr;
	
	/** callback to receive laser scanner readings*/
	void scan_callback( base::Time ts, const base::samples::LaserScan& scan_reading );

	/** callback to receive odometry readings from aggregator */
	void odometry_callback( base::Time ts, const wrappers::samples::RigidBodyState& odometry_reading );
	
	/** Transformation from robot body to odmetry frame */
	Eigen::Transform3d body2Odo;

	/** Transformation from laser to robot body frame 
	 *  Note, later on this will get dynamic */
	Eigen::Transform3d laser2Body;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Task(std::string const& name = "corridor_servoing::Task");

        RTT::NonPeriodicActivity* getNonPeriodicActivity();

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
        // bool startHook();

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
        void updateHook(std::vector<RTT::PortInterface*> const& updated_ports);
        

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

