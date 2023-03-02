#ifndef ARMMODEL_H
#define ARMMODEL_H

#include <vector>
#include "common/math/robotics.h"
#include "thirdparty/quadProgpp/QuadProg++.hh"

using namespace robo;

class ArmModel{
public:
    ArmModel(Vec3 endPosLocal, double endEffectorMass, Vec3 endEffectorCom, Mat3 endEffectorInertia);
    ~ArmModel(){};

    HomoMat forwardKinematics(Vec6 q, int index = 6);
    virtual bool inverseKinematics(HomoMat TDes, Vec6 qPast, Vec6& q_result, bool checkInWorkSpace = false);
    Mat6 CalcJacobianSpace(Vec6 q);
    Mat6 CalcJacobianBody(Vec6 q);
    Vec6 inverseDynamics(Vec6 q, Vec6 qd, Vec6 qdd, Vec6 Ftip);
    virtual void solveQP(Vec6 twist, Vec6 qPast, Vec6& qd_result, double dt) = 0;

    virtual bool checkInSingularity(Vec6 q) = 0;
    void jointProtect(Vec6& q, Vec6& qd);

    std::vector<double> getJointQMax() {return _jointQMax;}
    std::vector<double> getJointQMin() {return _jointQMin;}
    std::vector<double> getJointSpeedMax() {return _jointSpeedMax;}
    void addLoad(double load);
    bool checkAngle(Vec6 );

    const size_t dof = 6;
protected:
    void _buildModel();
    // initial parameters
    HomoMat _M; //End posture at the home position
    std::vector<HomoMat> _Mlist;// List of link frames {i} relative to {i-1} at the home position
    Vec3 _gravity;
    Mat6 _Slist;// spatial twist at home position
    std::vector<Mat6> _Glist;
    std::vector<Vec3> _jointAxis;// omega
    std::vector<Vec3> _jointPos;// p_0
    std::vector<double> _jointQMax;
    std::vector<double> _jointQMin;
    std::vector<double> _jointSpeedMax;
    Vec3 _endPosLocal; // based on mount postion
    Vec3 _endMountPosGlobal;

    std::vector<Vec3> _linkPosLocal;
    std::vector<double> _disJoint;//distance between joint
    std::vector<double> _linkMass;
    std::vector<Vec3> _linkCom; // center of mass
    std::vector<Mat3> _linkInertia;
    double _endEffectorMass;
    Vec3 _endEffectorCom;
    Mat3 _endEffectorInertia;

    double _load;
};

class Z1Model : public ArmModel{
public:
    Z1Model(size_t armType = 36, Vec3 endPosLocal = Vec3::Zero(), double endEffectorMass = 0.0,
            Vec3 endEffectorCom = Vec3::Zero(), Mat3 endEffectorInertia = Mat3::Zero());
    ~Z1Model(){};

    bool checkInSingularity(Vec6 q);
    bool inverseKinematics(HomoMat TDes, Vec6 qPast, Vec6& q_result, bool checkInWorkSpace = false);
    void solveQP(Vec6 twist, Vec6 qPast, Vec6& qd_result, double dt);
    double _theta2Bias;
private:
    void setParam_V3_6();
    void setParam_V3_7();
    void setParam_V3_8();
};

#endif