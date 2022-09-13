
#ifndef ARMDYNCKINEMODEL_H
#define ARMDYNCKINEMODEL_H

#include <rbdl/rbdl.h>
#include <vector>
#include "common/math/mathTypes.h"
#include "quadProgpp/include/QuadProg++.hh"

/* Kinematics only suitable for 6 DOF arm */
class ArmDynKineModel{
public:
    ArmDynKineModel(int dof, Vec3 endPosLocal,
        double endEffectorMass,
        Vec3 endEffectorCom, Mat3 endEffectorInertia);
    virtual ~ArmDynKineModel();

    Vec6 forDerivativeKinematice(Vec6 q, Vec6 qd);
    Vec6 invDerivativeKinematice(Vec6 q, Vec6 twistSpatial);
    Vec6 invDynamics(Vec6 q, Vec6 qd, Vec6 qdd, Vec6 endForce);

    int getDOF(){return _dof;}
    std::vector<double> getJointQMax(){return _jointQMax;}
    std::vector<double> getJointQMin(){return _jointQMin;}
    std::vector<double> getJointSpeedMax(){return _jointSpeedMax;}

    void solveQP(Vec6 twist, Vec6 pastAngle, Vec6 &result, double dt, bool setJ0QdZero, bool setQdZero);
    bool invKinematics(HomoMat gDes, Vec6 pastAngle, Vec6 &result, bool checkInWorkSpace = false);
    HomoMat forwardKinematics(Vec6 q, int index=6, bool isExp = false);   // index from 0 to 5, used for fourth version
    bool _useIKSolver = false;

protected:
    void _dofCheck();
    void _buildModel();

    int _dof;
    RigidBodyDynamics::Model *_model;
    std::vector<unsigned int> _linkID;

    std::vector<RigidBodyDynamics::Body> _body;
    std::vector<double> _linkMass;
    std::vector<Vec3> _linkCom;
    std::vector<RigidBodyDynamics::Math::Matrix3d> _linkInertia;
    double _endEffectorMass;
    Vec3 _endEffectorCom;
    Mat3 _endEffectorInertia;

    std::vector<RigidBodyDynamics::Joint> _joint;
    std::vector<Vec3> _jointAxis;
    std::vector<Vec3> _jointPos;
    std::vector<double> _jointQMax;
    std::vector<double> _jointQMin;
    std::vector<double> _jointSpeedMax;
    Vec3 _endPosGlobal, _endPosLocal;   // based on mount pos
    Vec3 _endMountPosGlobal, _endMountPosLocal;
    std::vector<Vec6> _theta;

    /* inverse derivative kinematics */
    RigidBodyDynamics::Math::MatrixNd _Jacobian;

    /* inverse dynamics */
    std::vector<RigidBodyDynamics::Math::SpatialVector> _endForceVec;

    /* inverse kinematics */
    bool _checkAngle(Vec6 angle);                   // check all angles once
    Vec6 JointQMax;
    Vec6 JointQMin;
    Vec6 JointQdMax;
    Vec6 JointQdMin;
    double theta0[2], theta1[4], theta2[4], theta3[4], theta4[4], theta5[4];
    double theta2Bias;

    std::vector<Vec3> _linkPosLocal;    
    std::vector<double> _disJoint;     
    double _disP13;                   
    Vec3 _vecP45;
    Vec3 _vecP14;
    Vec3 _vecP13;
    Vec3 _vecP34;

    Vec3 _joint4Axis;
    Vec3 _joint5Axis;

    Vec3 _plane324X, _plane324Z;        
    double _vec34ProjX, _vec34ProjZ;
    Vec3 _plane1234Y, _plane1234_X;     
    Vec3 _vecP04ProjXY;                 

    double _angle213, _angle123, _angle132;
    double _angle_x13;

    HomoMat _T46;

    /* quadprog */
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    Mat6 _G;
    Vec6 _g0;
    VecX _ci0;
    MatX _CI;
};

class Z1DynKineModel : public ArmDynKineModel{
public:
    Z1DynKineModel(Vec3 endPosLocal = Vec3::Zero(), 
        double endEffectorMass = 0.0,
        Vec3 endEffectorCom = Vec3::Zero(), 
        Mat3 endEffectorInertia = Mat3::Zero());
    ~Z1DynKineModel(){}

};


class Z1PlusDynKineModel : public ArmDynKineModel{
public:
    Z1PlusDynKineModel(Vec3 endPosLocal = Vec3::Zero(), 
        double endEffectorMass = 0.0,
        Vec3 endEffectorCom = Vec3::Zero(), 
        Mat3 endEffectorInertia = Mat3::Zero());
    ~Z1PlusDynKineModel(){}
};

#endif