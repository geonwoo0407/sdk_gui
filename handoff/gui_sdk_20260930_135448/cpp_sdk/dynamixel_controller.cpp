#include "dynamixel_controller.hpp"



Dxl_Controller::Dxl_Controller(Dxl *dxlPtr) : dxlPtr(dxlPtr)
{

}


// ************************************ GETTERS ***************************************** //

//Getter() : 관절각도[rad]
VectorXd Dxl_Controller::GetJointTheta()
{
    VectorXd th_(NUMBER_OF_DYNAMIXELS);
    th_ = dxlPtr->GetThetaAct();
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        th_cont[i] = th_[i];
    }
    return th_cont;
}

//Getter() : 관절각속도[rad/s]
VectorXd Dxl_Controller::GetThetaDot()
{
    VectorXd th_dot_(NUMBER_OF_DYNAMIXELS);
    th_dot_ = dxlPtr->GetThetaDot();
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)    
    {
        th_dot_cont[i] = th_dot_[i];
    }
    return th_dot_cont;
}

//Getter() : 각도의 차이와 이동평균필터를 이용해 각속도 계산 
VectorXd Dxl_Controller::GetThetaDotMAF()
{
    VectorXd a_th_dot(NUMBER_OF_DYNAMIXELS);
    a_th_dot = dxlPtr->GetThetaDotEstimated();
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        th_dot_cont[i] = a_th_dot[i];
    }
    MAF.topRows(Window_Size - 1) = MAF.bottomRows(Window_Size - 1);
    MAF.row(Window_Size - 1) = th_dot_cont.transpose();
    th_dot_MovAvgFilterd = MAF.colwise().mean();

    return th_dot_MovAvgFilterd;
}

//Getter() : Torque[Nm]
VectorXd Dxl_Controller::GetTorque()
{
    return torque_cont;
}

// **************************** SETTERS ******************************** //

//Setter() : 목표 Torque값 설정[Nm]
void Dxl_Controller::SetTorque(VectorXd tau)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) 
    {
        torque_cont[i] = tau[i];
    }
    dxlPtr->SetTorqueRef(torque_cont);
}

//Setter() : 목표 theta값 설정[rad]
bool Dxl_Controller::SetPosition(const VectorXd& theta)
{
    if (theta.size() != NUMBER_OF_DYNAMIXELS) return false;
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        th_cont[i] = theta[i];
    }
    dxlPtr->SetThetaRef(th_cont);
    return dxlPtr->syncWriteTheta();
}

bool Dxl_Controller::ConfigureTimeBasedProfile()
{
    return dxlPtr->ConfigureTimeBasedProfile();
}

bool Dxl_Controller::SetTimeBasedPosition(
    const VectorXd& theta, const std::vector<int>& motor_ids,
    uint32_t duration_ms, uint32_t acceleration_ms)
{
    if (theta.size() != NUMBER_OF_DYNAMIXELS) return false;
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; ++i) th_cont[i] = theta[i];
    return dxlPtr->syncWriteTimeBasedTheta(
        th_cont, motor_ids, duration_ms, acceleration_ms);
}

bool Dxl_Controller::SetTorqueEnabled(bool enabled)
{
    return dxlPtr->SetTorqueEnabled(enabled);
}
