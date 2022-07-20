#ifndef JOINT_H
#define JOINT_H

#include <vector>
#include "model/Motor.h"
#include "common/enumClass.h"

class Joint{
public:
    Joint(){}
    virtual ~Joint(){}
    void setJointKp (double jointKp);
    void setJointKd (double jointKd);
    void setJointQ  (double jointQ);
    void setJointDq (double jointDq);
    void setJointTau(double jointTau);

    double getJointQ();
    double getJointDq();
    double getJointDDq();
    double getJointTau();

    void getMotorQ(std::vector<double> &motorQ);
    void getMotorDq(std::vector<double> &motorDq);
    void getMotorDDq(std::vector<double> &motorDDq);
    void getMotorTau(std::vector<double> &motorTau);

    void getCmd(std::vector<MOTOR_send> &cmd);
    void setState(std::vector<MOTOR_recv> &state);
    void setCaliBias(double cali, std::vector<double> bias);

protected:
    JointMotorType _motorType;
    int _motorNum;
    double _jointInitAngle;
    std::vector<Motor*> _motor;
};

class Jointz1 : public Joint{
public:
    Jointz1(int motorID, double direction);
    Jointz1(int motorID0, int motorID1, double direction0, double direction1);
    Jointz1(int motorID, int motorMsgOrder, double direction);
    Jointz1(int motorID0, int motorID1, int motorMsgOrder0, int motorMsgOrder1, double direction0, double direction1);
    ~Jointz1(){}
};

class JointTigerHead : public Joint{
public:
    JointTigerHead(int motorID, double direction);
    JointTigerHead(int motorID, int motorMsgOrder, double direction);
    ~JointTigerHead(){}
};

#endif  // JOINT_H