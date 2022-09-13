#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/math/mathTypes.h"
#include "common/math/mathTools.h"
#include <vector>
#include <iostream>

struct LowlevelCmd{
private:
    size_t _dof = 6;
public:
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<double> kp;
    std::vector<double> kd;

    std::vector<std::vector<double>> q_cmd_data;
    std::vector<std::vector<double>> dq_cmd_data;
    std::vector<std::vector<double>> tau_cmd_data;

    LowlevelCmd();
    ~LowlevelCmd(){}

    void setZeroDq();
    void setZeroTau();
    void setZeroKp();
    void setZeroKd();
    void setControlGain();
    void setControlGain(std::vector<float> KP, std::vector<float> KW);
    void setQ(std::vector<double> qInput);
    void setQ(VecX qInput);
    void setQd(VecX qDInput);
    void setTau(VecX tauInput);
    void setPassive();
    void setGripperGain();
    void setGripperGain(float KP, float KW);
    void setGripperZeroGain();
    void setGripperQ(double qInput);
    double getGripperQ();
    void setGripperQd(double qdInput);
    double getGripperQd();
    void setGripperTau(double tauInput);
    double getGripperTau();
    Vec6 getQ();
    Vec6 getQd();

    void resizeGripper();
};


#endif  //LOWLEVELCMD_H
