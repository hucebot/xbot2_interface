#include "modelinterface2_pin.h"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

using namespace XBot;

VecConstRef ModelInterface2Pin::computeInverseDynamics() const
{
    if(!(_cached_computation & Rnea))
    {

        _tmp.rnea = pinocchio::rnea(_mdl, _data,
                                    getJointPosition(),
                                    getJointVelocity(),
                                    getJointAcceleration());


        _cached_computation |= Rnea;

    }

    return _tmp.rnea;
}


VecConstRef ModelInterface2Pin::computeInverseDynamics(
    const std::map<std::string, Eigen::Vector6d>& frame_forces) const
{
    // Create external forces vector (all zeros initially)
    pinocchio::container::aligned_vector<pinocchio::Force> fext(
        _mdl.njoints, pinocchio::Force::Zero()
    );

    
    // Set forces for specified joints
    for(const auto& [frame_name, force] : frame_forces)
    {
        pinocchio::FrameIndex frame_id = _mdl.getFrameId(frame_name);
        const pinocchio::Frame& frame_data = _mdl.frames[frame_id];

        pinocchio::JointIndex joint_id = frame_data.parentJoint;

        const pinocchio::SE3& jMf = frame_data.placement;

        pinocchio::Force f_contact(force.head(3), force.tail(3));

        fext[joint_id] += jMf.act(f_contact);

    }
    
    return pinocchio::rnea(_mdl, _data,
                                getJointPosition(),
                                getJointVelocity(),
                                getJointAcceleration(),
                                fext);
}


void ModelInterface2Pin::computeInverseDynamicsDerivative(Eigen::MatrixXd& dtau_dq, Eigen::MatrixXd& dtau_dv, Eigen::MatrixXd& dtau_da)
{
    //Maybe here it is possible to cache also ccrba!
    if(!(_cached_computation & dRnea))
    {
        pinocchio::computeRNEADerivatives(_mdl, _data,
                                          getJointPosition(),
                                          getJointVelocity(),
                                          getJointAcceleration());

        _data.M.triangularView<Eigen::StrictlyLower>() = _data.M.transpose().triangularView<Eigen::StrictlyLower>();

        _cached_computation |= dRnea;
    }

    dtau_dq = _data.dtau_dq;
    dtau_dv = _data.dtau_dv;

    dtau_da = _data.M;
}

// Convenience overload with named joint forces
void ModelInterface2Pin::computeInverseDynamicsDerivative(
        Eigen::MatrixXd& dtau_dq, 
        Eigen::MatrixXd& dtau_dv, 
        Eigen::MatrixXd& dtau_da,
        std::map<std::string, Eigen::MatrixXd>& dtau_dfext,
        const std::map<std::string, Eigen::Vector6d>& frame_forces)
{

    pinocchio::container::aligned_vector<pinocchio::Force> fext(_mdl.njoints, pinocchio::Force::Zero());
  

    for(const auto& [frame_name, force] : frame_forces)
    {
        pinocchio::FrameIndex frame_id = _mdl.getFrameId(frame_name);
        const pinocchio::Frame& frame_data = _mdl.frames[frame_id];


        pinocchio::JointIndex joint_id = frame_data.parentJoint;
        const pinocchio::SE3& jMf = frame_data.placement;

        pinocchio::Force f_contact(force.head(3), force.tail(3));

        fext[joint_id] += jMf.act(f_contact);
    }
    
    pinocchio::computeRNEADerivatives(_mdl, _data,
                                      getJointPosition(),
                                      getJointVelocity(),
                                      getJointAcceleration(),
                                      fext,
                                      dtau_dq,
                                      dtau_dv,
                                      dtau_da);
    
    dtau_da.triangularView<Eigen::StrictlyLower>() = 
        dtau_da.transpose().triangularView<Eigen::StrictlyLower>();


    for(const auto& [frame_name, force] : frame_forces)
    {

        pinocchio::FrameIndex frame_id = _mdl.getFrameId(frame_name);   
        pinocchio::computeFrameJacobian(_mdl, _data, getJointPosition(), frame_id, 
                                        pinocchio::LOCAL, dtau_dfext.at(frame_name));

        
    }
    


}


VecConstRef ModelInterface2Pin::computeGravityCompensation() const
{
    if(!(_cached_computation & Gcomp))
    {

        _tmp.gcomp = pinocchio::computeGeneralizedGravity(_mdl, _data,
                                                         getJointPosition());


        _cached_computation |= Gcomp;

    }

    return _tmp.gcomp;
}

VecConstRef ModelInterface2Pin::computeNonlinearTerm() const
{
    if(!(_cached_computation & NonlinearEffects))
    {
        _tmp.h = pinocchio::nonLinearEffects(_mdl, _data,
                                             getJointPosition(), getJointVelocity());

        _cached_computation |= NonlinearEffects;

    }

    return _tmp.h;
}
