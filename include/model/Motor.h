#ifndef MOTOR_H
#define MOTOR_H

#include <vector>
#include "common/enumClass.h"
#include "unitree_arm_sdk/common/arm_motor_common.h"

class Motor{
public:
    Motor(int id, int motorMsgOrder, double reductionRatio, MotorMountType type, double direction);
    virtual ~Motor(){}

    void setMotorKp (double jointKp);
    void setMotorKd (double jointKd);
    void setMotorQ  (double jointQ);
    void setMotorDq (double jointDq);
    void setMotorTau(double jointTau);

    double getJointQ(){return _qJoint;}
    double getJointDq(){return _dqJoint;}
    double getJointDDq(){return _ddqJoint;}
    double getJointTau(){return _tauJoint;}

    void getMotorQ(std::vector<double> &motorQ);
    void getMotorDq(std::vector<double> &motorDq);
    void getMotorDDq(std::vector<double> &motorDDq);
    void getMotorTau(std::vector<double> &motorTau);

    void setQBias(std::vector<double> qBias);
    void getCmd(std::vector<MOTOR_send> &cmd);
    void setState(std::vector<MOTOR_recv> &state);
protected:
    int _id;
    int _motorMsgOrder;
    double _kp, _kd, _q, _dq, _tau;
    double _qJoint, _dqJoint, _ddqJoint, _tauJoint;
    double _qMotor, _dqMotor, _ddqMotor, _tauMotor;
    double _reductionRatio;
    double _directon;
    double _qBias;
    MotorMountType _type;
};

class Motorz1 : public Motor{
public:
    Motorz1(int id, int motorMsgOrder, MotorMountType type, double direction);
    ~Motorz1(){}
};

class MotorGo1 : public Motor{
public:
    MotorGo1(int id, int motorMsgOrder, MotorMountType type, double direction);
    ~MotorGo1(){}
};

#endif  // MOTOR_H