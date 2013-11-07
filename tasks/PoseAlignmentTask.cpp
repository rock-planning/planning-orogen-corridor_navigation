/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseAlignmentTask.hpp"

using namespace corridor_navigation;

using namespace Eigen;

PoseAlignmentTask::PoseAlignmentTask(std::string const& name)
    : PoseAlignmentTaskBase(name)
{
}

PoseAlignmentTask::PoseAlignmentTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PoseAlignmentTaskBase(name, engine)
{
}

PoseAlignmentTask::~PoseAlignmentTask()
{
}

void PoseAlignmentTask::setBody2Odometry(const base::Time& ts)
{
    std::cout << "Got body2Odo" << std::endl;
    hasBody2Odometry |= _body2odometry.get(ts, body2Odometry);
}

void PoseAlignmentTask::setBody2World(const base::Time& ts)
{
    std::cout << "Got body2World" << std::endl;
    hasBody2World |= _body2world.get(ts, body2World);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseAlignmentTask.hpp for more detailed
// documentation about them.

bool PoseAlignmentTask::configureHook()
{
    if (! PoseAlignmentTaskBase::configureHook())
        return false;

    _body2odometry.registerUpdateCallback(boost::bind(&PoseAlignmentTask::setBody2Odometry, this, _1));
    _body2world.registerUpdateCallback(boost::bind(&PoseAlignmentTask::setBody2World, this, _1));
    
    return true;
}
bool PoseAlignmentTask::startHook()
{
    if (! PoseAlignmentTaskBase::startHook())
        return false;
    
    hasBody2Odometry = false;
    hasBody2World = false;
    hasTargetInOdometry = false;
 
    std::cout << "Started" << std::endl;
    return true;
}

void PoseAlignmentTask::updateHook()
{
    std::cout << "Update" << std::endl;
    PoseAlignmentTaskBase::updateHook();

    base::commands::Motion2D cmd;
    cmd.rotation = 0;
    cmd.translation = 0;

    if(!hasBody2World || !hasBody2Odometry)
    {
        std::cout << "No Transformations" << std::endl;

        _motion_commands.write(cmd);
        return;
    }        
    
    RTT::FlowStatus ret;
    base::Pose target_world;
    if((ret = _target_pose.readNewest(target_world)) == RTT::NoData)
    {
        std::cout << "No Target" << std::endl;

        _motion_commands.write(cmd);
        return;
    }

    

    if((ret == RTT::NewData) || ((ret == RTT::OldData) && !hasTargetInOdometry))
    {
        std::cout << "Got target" << std::endl;

        //note, the latest word coordinate frame is the 'target' frame
        const Affine3d target2Body(body2World.inverse());

        //calculate current target2Odometry transformation
        const Affine3d target2Odometry(body2Odometry * target2Body);

        //convert target into odometry coordinates
        target_odo.fromTransform(target2Odometry * target_world.toTransform());
        
        //got new pose, reinit
        curState = INIT;
        bestDistToTarget = std::numeric_limits< double >::max();
        hasTargetInOdometry = true;
    }

    if(!hasBody2Odometry || !hasTargetInOdometry)
    {
        
        _motion_commands.write(cmd);
        return;
    }

    

    //convert target into body coordinates
    base::Pose target_body;
    target_body.fromTransform(body2Odometry.inverse() * target_odo.toTransform());
    
    
    
    std::cout << "Target in Odo frame "<< target_odo.position.transpose() << " yaw " << target_odo.getYaw() << std::endl;
    std::cout << "Target in Body frame "<< target_body.position.transpose() << " yaw " << target_body.getYaw() << std::endl;

    //ignore z
    target_body.position.z() = 0;
    
    const double distToTarget = target_body.position.norm();
    
    bestDistToTarget = std::min(distToTarget, bestDistToTarget);
    
    switch(curState)
    {
        case INIT:
        {
            std::cout << "Dist to target is " << distToTarget << std::endl; 
            if(distToTarget < _min_distance_to_target.get())
            {
                curState = REACHED_TARGET_POSTION;
                std::cout << "REACHED_TARGET_POSTION" << std::endl;
                break;
            }

            double angleToPos = acos(target_body.position.normalized().dot(Vector3d::UnitX()));
            if(target_body.position.y() > 0)
                angleToPos *= -1;
            std::cout << "Angle to target position is " << angleToPos / M_PI * 180 << std::endl; 
            
            if(alignToAngle(angleToPos, cmd))
            {
                curState = ALIGNED_TO_TARGET_POSITION;
                break;
            }            
        }
            break;
        case ALIGNED_TO_TARGET_POSITION:
            if(distToTarget < _min_distance_to_target.get())
            {
                curState = REACHED_TARGET_POSTION;
                std::cout << "REACHED_TARGET_POSTION" << std::endl;
                break;
            }
            
            cmd.translation = _forward_speed.get();
            if(target_body.position.x() < 0)
                cmd.translation *= -1;

            if(fabs(bestDistToTarget - distToTarget) > _retry_distance.get() / 2.0 && 
                distToTarget > _retry_distance.get())
            {
                bestDistToTarget = std::numeric_limits< double >::max();
                curState = INIT;
            }
            
            break;
        case REACHED_TARGET_POSTION:
        {
            double angle = target_body.getYaw();
            if(alignToAngle(angle, cmd))
            {
                curState = REACHED_TARGET_POSTION_AND_ALIGNED;
                std::cout << "REACHED_TARGET_POSTION_AND_ALIGNED" << std::endl;
                break;
            }
        }
            break;
        case REACHED_TARGET_POSTION_AND_ALIGNED:
            break;
    }
    
    _motion_commands.write(cmd);
}

bool PoseAlignmentTask::alignToAngle(double angle, base::commands::Motion2D& cmd)
{
    if(fabs(angle) < _min_alginment_angle.get())
    {
        cmd.rotation = 0.0;
        return true;
    }

    std::cout << "Yaw diff is " << angle / M_PI * 180.0 << " Turning " << std::endl;
    cmd.rotation = _turn_speed.get();
        
    if(angle < 0)
    {
        cmd.rotation *= -1;
    }
    return false;
}

void PoseAlignmentTask::stopHook()
{
    PoseAlignmentTaskBase::stopHook();
}
void PoseAlignmentTask::cleanupHook()
{
    PoseAlignmentTaskBase::cleanupHook();
}
