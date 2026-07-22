#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include "step_dynamixel.hpp"

#define Window_Size     2   //이동평균필터의 관심 사이즈의 행

using Eigen::MatrixXd;

//controller -> cont

class Dxl_Controller
{
    public:
        //Construction
        Dxl_Controller(Dxl *dxlPtr);
        Dxl *dxlPtr;
        
        //Member Variable
        VectorXd th_cont = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_cont = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_MovAvgFilterd = VectorXd::Zero(NUMBER_OF_DYNAMIXELS); //Moving Average Filtered 
        MatrixXd MAF = MatrixXd::Zero(Window_Size, NUMBER_OF_DYNAMIXELS);
        VectorXd torque_cont = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);


        //Member Function
// ************************************ GETTERS ***************************************** //
        virtual VectorXd GetJointTheta();
        virtual VectorXd GetThetaDot();
        virtual VectorXd GetThetaDotMAF();
        virtual VectorXd GetTorque();
// **************************** SETTERS ******************************** //
        virtual void SetTorque(VectorXd tau);
        virtual bool SetPosition(const VectorXd& theta);
        virtual bool ConfigureTimeBasedProfile();
        virtual bool SetTimeBasedPosition(
            const VectorXd& theta, const std::vector<int>& motor_ids,
            uint32_t duration_ms, uint32_t acceleration_ms = 0);
        virtual bool SetTorqueEnabled(bool enabled);
        
};



#endif  // DYNAMIXEL_CONTROLLER_H
