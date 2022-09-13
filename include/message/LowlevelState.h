#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include <vector>
#include "common/math/mathTypes.h"
#include "common/math/mathTools.h"
#include "common/enumClass.h"
#include "common/utilities/LPFilter.h"
#include "message/UserValue.h"

struct LowlevelState{
private:
    size_t _dof = 6;
public:
    LowlevelState(double dt);
    ~LowlevelState();

    // MotorState motorState[12];
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> tau;

    std::vector<float> temperature;
    std::vector<uint8_t> errorstate;

    std::vector<double> qFiltered;
    std::vector<double> dqFiltered;
    std::vector<double> tauFiltered;

    std::vector<std::vector<double>> q_state_data;
    std::vector<std::vector<double>> dq_state_data;
    std::vector<std::vector<double>> tau_state_data;

    // ArmFSMStateName userCmd;
    UserValue userValue;
    LPFilter *qFilter;
    LPFilter *dqFilter;
    LPFilter *tauFilter;

    void resizeGripper(double dt);
    void runFilter();
    Vec6 getQ();
    Vec6 getQd();
    Vec6 getQdd();
    Vec6 getTau();
    Vec6 getQFiltered();
    Vec6 getQdFiltered();
    Vec6 getTauFiltered();
    double getGripperQ() {return q.at(q.size()-1);}
    double getGripperQd() {return dq.at(q.size()-1);}
    double getGripperTau() {return tau.at(tau.size()-1);}
    double getGripperTauFiltered() {return tauFiltered.at(tau.size()-1);}
};

#endif  //LOWLEVELSTATE_HPP
