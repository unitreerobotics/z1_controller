#ifndef USERVALUE_H
#define USERVALUE_H

#include "common/math/mathTypes.h"
#include "common/utilities/typeTrans.h"
#include <vector>

template<typename T>
std::vector<T> cutVector(std::vector<T> vec, size_t startID, size_t length){
    std::vector<T> result;
    result.assign(vec.begin()+startID, vec.begin()+startID+length);
    return result;
}

struct UserValue{
    Vec6 moveAxis;
    double gripperPos;

    void setData(std::vector<double> rawData){
        if(rawData.size() != 7){
            std::cout << "[ERROR] UserValue::setData, the size of rawDate is " << rawData.size() << " but not 7" << std::endl;
        }
        gripperPos = rawData.at(6);
        rawData = cutVector(rawData, 0, 6);
        moveAxis = typeTrans::getValue(rawData, moveAxis);
    }

    UserValue(){
        setZero();
    }
    void setZero(){
        moveAxis.setZero();
        gripperPos = 0;
        // gripperTau = 0;
    }
};

#endif