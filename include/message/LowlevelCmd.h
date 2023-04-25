#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/math/mathTypes.h"
#include "common/math/mathTools.h"
#include <vector>
#include <iostream>

struct LowlevelCmd{
public:
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<double> kp;
    std::vector<double> kd;

    std::vector<std::vector<double>> q_data; 
    std::vector<std::vector<double>> dq_data; 
    std::vector<std::vector<double>> tauf_data;
    std::vector<std::vector<double>> tau_data;

    LowlevelCmd();
    ~LowlevelCmd(){}

    void setZeroDq();
    void setZeroTau();
    void setZeroKp();
    void setZeroKd();
    void setQ(VecX qInput);
    void setQd(VecX qDInput);
    void setTau(VecX tauInput);

    void setControlGain();
    void setControlGain(std::vector<float> KP, std::vector<float> KW);
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
private:
    size_t _dof = 6;
};


#endif  //LOWLEVELCMD_H
