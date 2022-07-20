#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include <vector>
#include "common/math/mathTypes.h"
#include "common/math/mathTools.h"
#include "common/enumClass.h"
#include "common/utilities/LPFilter.h"
#include "message/UserValue.h"

// struct MotorState
// {
// 	unsigned int mode;
//     float q;
//     float dq;
//     float ddq;
//     float tauEst;

//     MotorState(){
//         q = 0;
//         dq = 0;
//         ddq = 0;
//         tauEst = 0;
//     }
// };

struct LowlevelState{
private:
    size_t _dof = 6;
public:
    LowlevelState(double dt){
#ifndef UNITREE_GRIPPER
        qFilter   = new LPFilter(dt, 3.0, _dof);
        dqFilter  = new LPFilter(dt, 3.0, _dof);
        tauFilter = new LPFilter(dt, 5.0, _dof);
        
        q.resize(_dof);
        dq.resize(_dof);
        ddq.resize(_dof);
        tau.resize(_dof);
        qFiltered.resize(_dof);
        dqFiltered.resize(_dof);
        tauFiltered.resize(_dof);
#endif
#ifdef UNITREE_GRIPPER
        qFilter   = new LPFilter(dt, 3.0, _dof+1);
        dqFilter  = new LPFilter(dt, 3.0, _dof+1);
        tauFilter = new LPFilter(dt, 5.0, _dof+1);

        q.resize(_dof+1);
        dq.resize(_dof+1);
        ddq.resize(_dof+1);
        tau.resize(_dof+1);
        qFiltered.resize(_dof+1);
        dqFiltered.resize(_dof+1);
        tauFiltered.resize(_dof+1);
#endif
    }
    ~LowlevelState(){
        delete qFilter;
        delete dqFilter;
        delete tauFilter;
    }

    // MotorState motorState[12];
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> tau;

    std::vector<double> qFiltered;
    std::vector<double> dqFiltered;
    std::vector<double> tauFiltered;

    // ArmFSMStateName userCmd;
    UserValue userValue;
    LPFilter *qFilter;
    LPFilter *dqFilter;
    LPFilter *tauFilter;

    void runFilter(){
        qFiltered = q;
	qFilter->addValue(qFiltered);

	dqFiltered = dq;
	dqFilter->addValue(dqFiltered);

        tauFiltered = tau;
        tauFilter->addValue(tauFiltered);
    }

    Vec6 getQ(){
        Vec6 qReturn;
        for(int i(0); i < 6; ++i){
            qReturn(i) = q.at(i);
        }
        return qReturn;
    }

    Vec6 getQd(){
        Vec6 qdReturn;
        for(int i(0); i < 6; ++i){
            qdReturn(i) = dq.at(i);
        }
        return qdReturn;
    }

    Vec6 getQdd(){
        Vec6 qddReturn;
        for(int i(0); i < 6; ++i){
            qddReturn(i) = ddq.at(i);
        }
        return qddReturn;
    }

    Vec6 getTau(){
        Vec6 tauReturn;
        for(int i(0); i < 6; ++i){
            tauReturn(i) = tau.at(i);
        }
        return tauReturn;
    }

    Vec6 getQFiltered(){
        Vec6 qReturn;
        for(int i(0); i < 6; ++i){
            qReturn(i) = qFiltered.at(i);
        }
        return qReturn;
    }

    Vec6 getQdFiltered(){
        Vec6 dqReturn;
        for(int i(0); i < 6; ++i){
            dqReturn(i) = dqFiltered.at(i);
        }
        return dqReturn;
    }


    Vec6 getTauFiltered(){
        Vec6 tauReturn;
        for(int i(0); i < 6; ++i){
            tauReturn(i) = tauFiltered.at(i);
        }
        return tauReturn;
    }

    double getGripperQ(){
        return q.at(q.size()-1);
    }

    double getGripperQd(){
        return dq.at(q.size()-1);
    }

    double getGripperTau(){
        return tau.at(tau.size()-1);
    }

    double getGripperTauFiltered(){
        return tauFiltered.at(tau.size()-1);
    }
};

#endif  //LOWLEVELSTATE_HPP
