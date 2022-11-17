#ifndef __KDTREEPOINT_H__
#define __KDTREEPOINT_H__

#include <Eigen/Dense>

/**
 * KDtree point class
 */

class KDTreePoint
{

public:
    KDTreePoint(VectorXf &mean, int id) : mean(mean), id(id) {}

    double operator[](const int index) const {
        VectorXf mean = this->mean;
        
        double val = mean(0);

        if (index == 0)
            return this->mean(0);
        else if (index == 1)
            return this->mean(1);
        else 
            throw std::invalid_argument("Index must be 0 or 1");
    };
    
    int getId() {
        return this->id;
    };

    VectorXf& getMean() {
        return this->mean;
    }

    static const int DIM = 2;

private:
    VectorXf mean;
    int id;
};

#endif