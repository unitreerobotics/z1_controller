#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <iostream>
#include "common/math/mathTypes.h"

// template<typename T1, typename T2>
// inline T1 max(const T1 a, const T2 b){
// 	return (a > b ? a : b);
// }

// template<typename T1, typename T2>
// inline T1 min(const T1 a, const T2 b){
// 	return (a < b ? a : b);
// }

template<typename T>
T max(T value){
    return value;
}

template<typename T, typename... Args>
inline T max(const T val0, const Args... restVal){
    return val0 > (max<T>(restVal...)) ? val0 : max<T>(restVal...);
}

template<typename T>
T min(T value){
    return value;
}

template<typename T, typename... Args>
inline T min(const T val0, const Args... restVal){
    return val0 > (min<T>(restVal...)) ? val0 : min<T>(restVal...);
}

inline Mat2 skew(const double& w){
    Mat2 mat; mat.setZero();
    mat(0, 1) = -w;
    mat(1, 0) =  w;
    return mat;
}

inline Mat3 skew(const Vec3& v) {
    Mat3 m;
    m << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return m;
}

enum class TurnDirection{
    NOMATTER,
    POSITIVE,
    NEGATIVE
};

/* first - second */
inline double angleError(double first, double second, TurnDirection direction = TurnDirection::NOMATTER){
    double firstMod = fmod(first, 2.0*M_PI);
    double secondMod = fmod(second, 2.0*M_PI);

    if(direction == TurnDirection::POSITIVE){
        if(firstMod - secondMod < 0.0){
            return 2*M_PI + firstMod - secondMod;
        }else{
            return firstMod - secondMod;
        }
    }
    else if(direction == TurnDirection::NEGATIVE){
        if(firstMod - secondMod > 0.0){
            return -2*M_PI + firstMod - secondMod;
        }else{
            return firstMod - secondMod;
        }
    }else{//no matter
        if(fabs(firstMod - secondMod) > fabs(secondMod - firstMod)){
            return secondMod - firstMod;
        }else{
            return firstMod - secondMod;
        }        
    }
}

/* firstVec - secondVec */
inline VecX angleError(VecX firstVec, VecX secondVec, TurnDirection directionMatter = TurnDirection::NOMATTER){
    if(firstVec.rows() != secondVec.rows()){
        std::cout << "[ERROR] angleError, the sizes of firstVec and secondVec are different!" << std::endl;
    }

    VecX result = firstVec;
    for(int i(0); i<firstVec.rows(); ++i){
        result(i) = angleError(firstVec(i), secondVec(i), directionMatter);
    }

    return result;
}

inline bool vectorEqual(VecX v1, VecX v2, double tolerance){
    if(v1.rows() != v2.rows()){
        std::cout << "[WARNING] vectorEqual, the size of two vectors is not equal, v1 is "
            << v1.rows() << ", v2 is " << v2.rows() << std::endl;
        return false;
    }
    for(int i(0); i<v1.rows(); ++i){
        if(fabs(v1(i)-v2(i))>tolerance){
            return false;
        }
    }
    return true;
}

inline bool inInterval(double value, double limValue1, double limValue2, bool canEqual1 = false, bool canEqual2 = false){
    double lowLim, highLim;
    bool lowEqual, highEqual;
    if(limValue1 >= limValue2){
        highLim   = limValue1;
        highEqual = canEqual1;
        lowLim    = limValue2;
        lowEqual  = canEqual2;
    }else{
        lowLim    = limValue1;
        lowEqual  = canEqual1;
        highLim   = limValue2;
        highEqual = canEqual2;
    }

    if((value > highLim) || (value < lowLim)){
        return false;
    }
    if((value == highLim) && !highEqual){
        return false;
    }
    if((value == lowLim) && !lowEqual){
        return false;
    }

    return true;
}

inline double saturation(const double a, double limValue1, double limValue2){
    double lowLim, highLim;
    if(limValue1 >= limValue2){
        lowLim = limValue2;
        highLim= limValue1;
    }else{
        lowLim = limValue1;
        highLim= limValue2;
    }

    if(a < lowLim){
        return lowLim;
    }
    else if(a > highLim){
        return highLim;
    }
    else{
        return a;
    }
}

inline double saturation(const double a, Vec2 limits){
    return saturation(a, limits(0), limits(1));
}

template<typename T0, typename T1>
inline T0 killZeroOffset(T0 a, const T1 limit){
    if((a > -limit) && (a < limit)){
        a = 0;
    }
    return a;
}

template<typename T0, typename T1, typename T2>
inline T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

// x: [0, 1], windowRatio: (0, 0.5)
template<typename T>
inline T windowFunc(const T x, const T windowRatio, const T xRange=1.0, const T yRange=1.0){
    if((x < 0)||(x > xRange)){
        // printf("[ERROR] The x of function windowFunc should between [0, xRange]\n");
        std::cout << "[ERROR][windowFunc] The x=" << x << ", which should between [0, xRange]" << std::endl;
    }
    if((windowRatio <= 0)||(windowRatio >= 0.5)){
        // printf("[ERROR] The windowRatio of function windowFunc should between (0, 0.5)\n");
        std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio << ", which should between [0, 0.5]" << std::endl;
    }

    if(x/xRange < windowRatio){
        return x * yRange / (xRange * windowRatio);
    }
    else if(x/xRange > 1 - windowRatio){
        return yRange * (xRange - x)/(xRange * windowRatio);
    }
    else{
        return yRange;
    }
}

template<typename T1, typename T2>
inline void updateAverage(T1 &exp, T2 newValue, double n){
    if(exp.rows()!=newValue.rows()){
        std::cout << "The size of updateAverage is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.001){
        exp = newValue;
    }else{
        exp = exp + (newValue - exp)/n;
    }
}

template<typename T1, typename T2, typename T3>
inline void updateCovariance(T1 &cov, T2 expPast, T3 newValue, double n){
    if( (cov.rows()!=cov.cols()) || (cov.rows() != expPast.rows()) || (expPast.rows()!=newValue.rows())){
        std::cout << "The size of updateCovariance is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.1){
        cov.setZero();
    }else{
        cov = cov*(n-1)/n + (newValue-expPast)*(newValue-expPast).transpose()*(n-1)/(n*n);
    }
}

template<typename T1, typename T2, typename T3>
inline void updateAvgCov(T1 &cov, T2 &exp, T3 newValue, double n){
    // The order matters!!! covariance first!!!
    updateCovariance(cov, exp, newValue, n);
    updateAverage(exp, newValue, n);
}

/* rotate matrix about x axis */
inline RotMat rotX(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << 1, 0, 0, 0, c, -s, 0, s, c;
    return R;
}

/* rotate matrix about y axis */
inline RotMat rotY(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, 0, s, 0, 1, 0, -s, 0, c;
    return R;
}

/* rotate matrix about z axis */
inline RotMat rotZ(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

inline RotMat so3ToRotMat(const Vec3& _rot){
    RotMat R;
    double theta = _rot.norm();
    if (fabs(theta) <1e-6)
        R = RotMat::Identity();
    else{
        Vec3  u_axis(_rot/theta);
        double cos_theta = cos(theta);
        R = cos_theta*RotMat::Identity()+sin(theta)*skew(u_axis) +(1-cos_theta)*(u_axis*u_axis.transpose());
    }
    return R;
}

/* row pitch yaw to rotate matrix */
inline RotMat rpyToRotMat(const double& row, const double& pitch, const double& yaw) {
    RotMat m = rotZ(yaw) * rotY(pitch) * rotX(row);
    return m;
}

inline RotMat quatToRotMat(const Quat& q) {
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);

    RotMat R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}

inline Vec3 rotMatToRPY(const Mat3& R) {
    Vec3 rpy;
    rpy(0) = atan2(R(2,1),R(2,2));//asin(R(2,1)/cos(rpy(1))); // atan2(R(2,1),R(2,2));
    rpy(1) = asin(-R(2,0));
    rpy(2) = atan2(R(1,0),R(0,0));
    return rpy;
}

/* rotate matrix to exponential coordinate(omega*theta) */
inline Vec3 rotMatToExp(const RotMat& rm){
    double cosValue = rm.trace()/2.0-1/2.0;
    if(cosValue > 1.0f){
        cosValue = 1.0f;
    }else if(cosValue < -1.0f){
        cosValue = -1.0f;
    }

    double angle = acos(cosValue);
    Vec3 exp;
    if (fabs(angle) < 1e-5){
        exp=Vec3(0,0,0);
    }
    else if (fabs(angle - M_PI) < 1e-5){
        exp = angle * Vec3(rm(0,0)+1, rm(0,1), rm(0,2)) / sqrt(2*(1+rm(0, 0)));
    }
    else{
        exp=angle/(2.0f*sin(angle))*Vec3(rm(2,1)-rm(1,2),rm(0,2)-rm(2,0),rm(1,0)-rm(0,1));
    }
    return exp;
}

/* add 1 at the end of Vec3 */
inline Vec4 homoVec(Vec3 v3, double end = 1.0){
    Vec4 v4;
    v4.block(0, 0, 3, 1) = v3;
    v4(3) = end;
    return v4;
}

/* remove 1 at the end of Vec4 */
inline Vec3 noHomoVec(Vec4 v4){
    Vec3 v3;
    v3 = v4.block(0, 0, 3, 1);
    return v3;
}

/* build Homogeneous Matrix by p and m */
inline HomoMat homoMatrix(Vec3 p, Mat3 m){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = m;
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}

/* build Homogeneous Matrix by p and w */
inline HomoMat homoMatrix(Vec3 p, Vec3 w){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = so3ToRotMat(w);
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}

/* build Homogeneous Matrix by x axis, y axis and p */
inline HomoMat homoMatrix(Vec3 x, Vec3 y, Vec3 p){
    HomoMat homoM;
    homoM.setZero();
    Vec3 xN = x.normalized();
    Vec3 yN = y.normalized();
    homoM.block(0, 0, 3, 1) = xN;
    homoM.block(0, 1, 3, 1) = yN;
    homoM.block(0, 2, 3, 1) = skew(xN) * yN;
    homoM.topRightCorner(3, 1) = p;
    homoM(3, 3) = 1;
    return homoM;
}

/* build Homogeneous Matrix of rotate joint */
inline HomoMat homoMatrixRotate(Vec3 q, Vec3 w){
    HomoMat homoM;
    Mat3 eye3 = Mat3::Identity();
    Mat3 rotateM = so3ToRotMat(w);
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = rotateM;
    homoM.topRightCorner(3, 1) = (eye3 - rotateM) * q;
    // homoM.topRightCorner(3, 1) = q;
    homoM(3, 3) = 1;

    return homoM;
}

/* build Homogeneous Matrix by posture */
inline HomoMat homoMatrixPosture(Vec6 posture){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = rpyToRotMat(posture(0),posture(1),posture(2));
    homoM.topRightCorner(3, 1) << posture(3),posture(4),posture(5);
    homoM(3, 3) = 1;
    return homoM;
}

/* inverse matrix of homogeneous matrix */
inline HomoMat homoMatrixInverse(HomoMat homoM){
    HomoMat homoInv;
    homoInv.setZero();
    homoInv.topLeftCorner(3, 3) = homoM.topLeftCorner(3, 3).transpose();
    homoInv.topRightCorner(3, 1) = -homoM.topLeftCorner(3, 3).transpose() * homoM.topRightCorner(3, 1);
    homoInv(3, 3) = 1;
    return homoInv;
}

/* get position of Homogeneous Matrix */
inline Vec3 getHomoPosition(HomoMat mat){
    return mat.block(0, 3, 3, 1);
}

/* get rotate matrix of Homogeneous Matrix */
inline RotMat getHomoRotMat(HomoMat mat){
    return mat.block(0, 0, 3, 3);
}

/* convert homogeneous matrix to posture vector */
inline Vec6 homoToPosture(HomoMat mat){
    Vec6 posture;
    posture.block(0,0,3,1) = rotMatToRPY(getHomoRotMat(mat));
    posture.block(3,0,3,1) = getHomoPosition(mat);
    return posture;
}

/* convert posture vector matrix to homogeneous */
inline HomoMat postureToHomo(Vec6 posture){
    HomoMat homoM;
    homoM.setZero();
    homoM.topLeftCorner(3, 3) = rpyToRotMat(posture(0), posture(1), posture(2));
    homoM.topRightCorner(3, 1) << posture(3), posture(4), posture(5);
    homoM(3, 3) = 1;
    return homoM;
}

// Calculate average value and covariance
class AvgCov{
public:
    AvgCov(unsigned int size, std::string name, bool avgOnly=false, unsigned int showPeriod=1000, unsigned int waitCount=5000, double zoomFactor=10000)
            :_size(size), _showPeriod(showPeriod), _waitCount(waitCount), _zoomFactor(zoomFactor), _valueName(name), _avgOnly(avgOnly) {
        _exp.resize(size);
        _cov.resize(size, size);
        _defaultWeight.resize(size, size);
        _defaultWeight.setIdentity();
        _measureCount = 0;
    }
    void measure(VecX newValue){
        ++_measureCount;

        if(_measureCount > _waitCount){
            updateAvgCov(_cov, _exp, newValue, _measureCount-_waitCount);
            if(_measureCount % _showPeriod == 0){
            // if(_measureCount < _waitCount + 5){
                std::cout << "******" << _valueName << " measured count: " << _measureCount-_waitCount << "******" << std::endl;
                // std::cout << _zoomFactor << " Times newValue of " << _valueName << std::endl << (_zoomFactor*newValue).transpose() << std::endl;
                std::cout << _zoomFactor << " Times Average of " << _valueName << std::endl << (_zoomFactor*_exp).transpose() << std::endl;
                if(!_avgOnly){
                    std::cout << _zoomFactor << " Times Covariance of " << _valueName << std::endl << _zoomFactor*_cov << std::endl;
                }
            }
        }
    }
private:
    VecX _exp;
    MatX _cov;
    MatX _defaultWeight;
    bool _avgOnly;
    unsigned int _size;
    unsigned int _measureCount;
    unsigned int _showPeriod;
    unsigned int _waitCount;
    double _zoomFactor;
    std::string _valueName;
};
#endif