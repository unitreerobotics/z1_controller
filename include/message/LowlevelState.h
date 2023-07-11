#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include <vector>
#include "common/math/mathTools.h"
#include "common/enumClass.h"
#include "common/math/Filter.h"

struct LowlevelState{
public:
    LowlevelState(double dt);
    ~LowlevelState();

    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> tau;

    std::vector<std::vector<double>> q_data; 
    std::vector<std::vector<double>> dq_data; 
    std::vector<std::vector<double>> ddq_data;
    std::vector<std::vector<double>> tau_data;

    std::vector<int> temperature;
    std::vector<uint8_t> errorstate;
    std::vector<uint8_t> isMotorConnected;

    std::vector<double> qFiltered;
    std::vector<double> dqFiltered;
    std::vector<double> ddqFiltered;
    std::vector<double> tauFiltered;

    LPFilter *qFilter;
    LPFilter *dqFilter;
    LPFilter *ddqFilter;
    LPFilter *tauFilter;

    void resizeGripper(double dt);
    void runFilter();
    bool checkError();
    Vec6 getQ();
    Vec6 getQd();
    Vec6 getQdd();
    Vec6 getTau();
    Vec6 getQFiltered();
    Vec6 getQdFiltered();
    Vec6 getQddFiltered();
    Vec6 getTauFiltered();
    double getGripperQ();
    double getGripperQd();
    double getGripperTau();
    double getGripperTauFiltered();
private:
    size_t _dof = 6;
    int temporatureLimit = 80;// centigrade
    std::vector<int> _isMotorConnectedCnt;
    std::vector<bool> _isMotorLostConnection;
};

#endif  //LOWLEVELSTATE_HPP
