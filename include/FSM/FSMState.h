#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "common/math/mathTools.h"
#include "common/utilities/timer.h"
#include "FSM/BaseState.h"
#include "model/unitree_gripper.h"

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
    void _tauFriction();
    void _zero_position_joint4_protection();

    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    IOInterface *_ioInter;
    ArmModel *_armModel;
    std::shared_ptr<Unitree_Gripper> _gripper;

    Vec6 _qPast, _qdPast, _q, _qd, _qdd, _tauForward;
    double _gripperPos, _gripperW, _gripperTau;

    CtrlComponents *_ctrlComp;
    Vec6 _g, _tauCmd, _tauFric;

private:

    uint _collisionCnt;

    Vec6 _mLinearFriction;
    Vec6 _mCoulombFriction;

};

#endif  // FSMSTATE_H
