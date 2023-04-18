#ifndef SCURVE_H
#define SCURVE_H

/* 归一化后的s曲线 */

#include <iostream>

class SCurve{
public:
    SCurve(){}
    ~SCurve(){}
    void setSCurve(double deltaQ, double dQMax, double ddQMax, double dddQMax);
    void restart();
/*
考虑到归一化，在实际使用时，物理量为：
加速度 = deltaQ * getDDs();
*/
    double getDDs();
    double getDDs(double t);
/*
考虑到归一化，在实际使用时，物理量为：
速度 = deltaQ * getDs();
*/
    double getDs();
    double getDs(double t);
/*
考虑到归一化，在实际使用时，物理量为：
位置 = deltaQ * gets();
*/
    double gets();
    double gets(double t);

    double getT();
private:
    int _getSegment(double t);
    void _setFunc();
    double _runTime();

    bool _started = false;
    double _startTime;

    double _J, _aMax, _vMax;
    double _T[7];   // period
    double _t[7];   // moment
    double _v0, _v1;        // ds at _t[0], _t[1]
    double _s0, _s1, _s2, _s3, _s4, _s5, _s6;   // s at _t[0], _t[1]
};

#endif  // SCURVE_H