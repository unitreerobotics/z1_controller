#ifndef CartesionSpaceTraj_H
#define CartesionSpaceTraj_H

#include <vector>
#include <trajectory/Trajectory.h>
#include <string>
#include "control/CtrlComponents.h"

class CartesionSpaceTraj : public Trajectory{
public:
    CartesionSpaceTraj(CtrlComponents *ctrlComp);
    CartesionSpaceTraj(ArmDynKineModel *armModel);
    CartesionSpaceTraj(ArmDynKineModel *armModel, CSVTool *csvState);
    ~CartesionSpaceTraj(){}

    void setCartesionTraj(Vec6 startP, Vec6 endP, double oriSpeed, double posSpeed);
    void setCartesionTraj(Vec6 startP, Vec6 middleP, double middleS, Vec6 endP, double oriSpeed, double posSpeed);
    bool getCartesionCmd(Vec6 pastPosture, Vec6 &endPosture, Vec6 &endTwist);

    void setCartesionTrajMoveC(Vec6 startP, Vec6 middleP, Vec6 endP, double oriSpeed, double posSpeed);
    bool getCartesionCmdMoveC(Vec6 pastPosture, Vec6 &endPosture, Vec6 &endTwist);

    double _radius;

private:
    void _centerCircle(Vec3 p1, Vec3 p2, Vec3 p3);
    bool _checkPostureValid(Vec6 endPosture, int pointOrder);
    void _generateA345();

    double _pathTimeTemp;       // path total time
    double _s, _sDot;       // [0, 1]
    double _a3, _a4, _a5;   // parameters of path

    int _trajOrder;     // The order of trajectory curve
    std::vector<Vec6> _curveParam;

    /* MoveC instruct */
    Vec3 _center;
    double _theta;
    Vec3 _axis_x;
    Vec3 _axis_y;
    Vec3 _axis_z;
};

#endif  // JOINTSPACETRAJ_H