#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "common/math/mathTypes.h"
#include "control/CtrlComponents.h"
#include "unitree_arm_sdk/timer.h"

class Trajectory{
public:
    Trajectory(CtrlComponents *ctrlComp){
        _ctrlComp = ctrlComp;
        _armModel = ctrlComp->armModel;
        _csvState = ctrlComp->stateCSV;
        _dof = _armModel->getDOF();
        _hasKinematic = true;
        _jointMaxQ = _armModel->getJointQMax();
        _jointMinQ = _armModel->getJointQMin();
        _jointMaxSpeed = _armModel->getJointSpeedMax();
    }
    Trajectory(ArmDynKineModel *armModel){
        _ctrlComp = nullptr;
        _armModel = armModel;
        _csvState = nullptr;
        _dof = armModel->getDOF();
        if(_dof == 6){
            _hasKinematic = true;
        }
        _jointMaxQ = _armModel->getJointQMax();
        _jointMinQ = _armModel->getJointQMin();
        _jointMaxSpeed = _armModel->getJointSpeedMax();
    }
    Trajectory(ArmDynKineModel *armModel, CSVTool *csvState):Trajectory(armModel){
        _csvState = csvState;
    }
    virtual ~Trajectory(){}
    virtual bool getJointCmd(Vec6 &q, Vec6 &qd){return false;};
    virtual bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd){return false;};
    virtual bool getCartesionCmd(Vec6 pastPosture, Vec6 &endPosture, Vec6 &endTwist){return false;};

    void restart(){
        _pathStarted = false;
        _reached = false;
        _qPast = _startQ;
    }
    virtual void setGripper(double startQ, double endQ){
        _gripperStartQ = startQ;
        _gripperEndQ   = endQ;
    }
    bool correctYN(){return _settingCorrect;}
    Vec6 getStartQ(){return _startQ;}
    Vec6 getEndQ(){return _endQ;}
    double getEndGripperQ(){return _gripperEndQ;};
    double getStartGripperQ(){return _gripperStartQ;};
    HomoMat getStartHomo(){
        if(_hasKinematic){
            return _startHomo;
        }else{
            std::cout << "[ERROR] This trajectory do not have kinematics" << std::endl;
            exit(-1);
        }
    }
    HomoMat getEndHomo(){
        if(_hasKinematic){
            return _endHomo;
        }else{
            std::cout << "[ERROR] This trajectory do not have kinematics" << std::endl;
            exit(-1);
        }
    }

    Vec6 getEndPosture(){
        if(_hasKinematic){
            return _endPosture;
        }else{
            std::cout << "[ERROR] This trajectory do not have kinematics" << std::endl;
            exit(-1);
        }
    }
    double getPathTime(){return _pathTime;}
protected:
    CtrlComponents *_ctrlComp;
    ArmDynKineModel *_armModel;
    CSVTool *_csvState;
    bool _pathStarted = false;
    bool _reached = false;
    bool _paused = false;
    bool _settingCorrect = true;
    double _startTime;
    double _currentTime;
    double _pathTime;
    double _tCost;

    Vec6 _qPast;
    Vec6 _startPosture, _endPosture;
    Vec6 _startQ, _endQ;
    HomoMat _startHomo, _endHomo;

    double _gripperStartQ;
    double _gripperEndQ;

    size_t _dof;
    bool _hasKinematic;

    std::vector<double> _jointMaxQ;
    std::vector<double> _jointMinQ;
    std::vector<double> _jointMaxSpeed;

    void _runTime(){
        _currentTime = getTimeSecond();

        if(!_pathStarted){
            _pathStarted = true;
            _startTime = _currentTime;
            _tCost = 0;
        }

        _tCost = _currentTime - _startTime;

        _reached = (_tCost>_pathTime) ? true : false;
        _tCost = (_tCost>_pathTime) ? _pathTime : _tCost;
    }
};

#endif  // TRAJECTORY_H