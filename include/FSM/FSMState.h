#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "common/math/mathTools.h"
#include "common/utilities/timer.h"
#include "FSM/BaseState.h"

class FSMState : public BaseState{
public:
    FSMState(CtrlComponents *ctrlComp, ArmFSMStateName stateName, std::string stateNameString);
    virtual ~FSMState(){}

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual int checkChange(int cmd) {return (int)ArmFSMStateName::INVALID;}
    bool _collisionTest();
    
protected:
    void _armCtrl();
    void _recordData();
    Vec6 _postureToVec6(Posture posture);

    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    IOInterface *_ioInter;
    ArmModel *_armModel;

    Vec6 _qPast, _qdPast, _q, _qd, _qdd, _tauf, _tauCmd, _g;
    double _gripperPos, _gripperW, _gripperTau;
    uint _collisionCnt;

    CtrlComponents *_ctrlComp;
};

#endif  // FSMSTATE_H
