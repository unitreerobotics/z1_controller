#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/math/mathTools.h"
#include "common/math/mathTypes.h"
#include "unitree_arm_sdk/timer.h"
#include "model/ArmDynKineModel.h"

#include "FSM/FiniteStateMachine.h"

class FSMState : public BaseState{
public:
    FSMState(CtrlComponents *ctrlComp, ArmFSMStateName stateName, std::string stateNameString);
    virtual ~FSMState(){}

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual int checkChange(int cmd) {return (int)ArmFSMStateName::INVALID;}
    
protected:
    void _armCtrl();
    void _tauDynForward();
    void _tauFriction();
    void _qdFeedBack();
    bool _collisionTest();
    void _jointPosProtect();
    void _jointSpeedProtect();
    void _stateUpdate();
    
    void _gripperCmd();
    void _gripperCtrl();
    double _gripperPos;
    double _gripperW;
    double _gripperTau;
    double _gripperPosStep;//keyboard
    double _gripperTauStep;

    double _gripperLinearFriction;
    double _gripperCoulombFriction;
    static double _gripperCoulombDirection;
    double _gripperFriction;

    CtrlComponents *_ctrlComp;
    ArmFSMStateName _nextStateName;

    IOInterface *_ioInter;
    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    UserValue _userValue;
    ArmDynKineModel *_armModel;

    Vec6 _qPast, _qdPast, _q, _qd, _qdd;
    Vec6 _tau, _endForce;

    Vec6 _coulombFriction;
    static Vec6 _coulombDirection;
    Vec6 _linearFriction;

    Vec6 _kpDiag, _kdDiag;
    Mat6 _Kp, _Kd;

    std::vector<double> _jointQMax;
    std::vector<double> _jointQMin;
    std::vector<double> _jointSpeedMax;

    SendCmd _sendCmd;
    RecvState _recvState;
};

#endif  // FSMSTATE_H
