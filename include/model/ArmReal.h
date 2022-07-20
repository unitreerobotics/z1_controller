#ifndef ARM_H
#define ARM_H

#include <vector>
#include "common/math/mathTypes.h"
#include "model/Joint.h"
#include "model/Motor.h"
#include "unitree_arm_sdk/udp.h"

class ArmReal{
public:
    ArmReal(int dof, int motorNum, IOPort *ioPort);
    virtual ~ArmReal();
    void setJointKp(std::vector<double> jointKp);
    void setJointKd(std::vector<double> jointKd);
    void setJointQ(std::vector<double> jointQ);
    void setJointDq(std::vector<double> jointDq);
    void setJointTau(std::vector<double> jointTau);

    void getJointQ(std::vector<double> &jointQState);
    void getJointDq(std::vector<double> &jointDqState);
    void getJointDDq(std::vector<double> &jointDDqState);
    void getJointTau(std::vector<double> &jointTauState);

    void getMotorQ(std::vector<double> &motorQ);
    void getMotorDq(std::vector<double> &motorDq);
    void getMotorTau(std::vector<double> &motorTau);

    void armCalibration();
    void armCalibration(std::vector<double> motorAngleBias);
    bool armComm();
    size_t getDof(){return _dof;}
    size_t getMotorNum(){return _motorNum;}
    // int  getMotorNum(){return _motorNum;}

protected:
    void _init();
    void _getCmd(std::vector<MOTOR_send> &cmd);
    void _setState(std::vector<MOTOR_recv> &motorState);
    void _getMotorQBias();
    // void _setComm();
    void _setCaliBias();

    int _motorNum;
    int _dof;
    int _commReturn;
    bool _commYN, _motorCommYN;
    std::vector<Joint*> _joints;
    std::vector<double> _jointCaliAngle;
    std::vector<double> _motorAngleBias;
    std::vector<MOTOR_send> _motorCmd;
    std::vector<MOTOR_recv> _motorState;

    /* serial or UDP Communication */
    IOPort *_ioPort;
};

class z1Real : public ArmReal{
public:
    z1Real(IOPort *ioPort);
    ~z1Real(){}
};

class TigerHeadReal : public ArmReal{
public:
    TigerHeadReal(IOPort *ioPort);
    ~TigerHeadReal(){}
};

class DogHeadReal : public ArmReal{
public:
    DogHeadReal(IOPort *ioPort);
    ~DogHeadReal(){}
};

class TestSerial : public ArmReal{
public:
    TestSerial(IOPort *ioPort);
    ~TestSerial(){}
};

#endif  // ARM_H