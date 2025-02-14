/** /file dab_spring_solver_euler.h
*/

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "dab_singleton.h"

namespace dab
{

namespace spring
{
    
#pragma mark EulerSolver Definition

class EulerSolver : public Singleton< EulerSolver >
{
public:
    EulerSolver();
    ~EulerSolver();
    
    void setTimeStep( float pTimeStep );
    
    template< unsigned int Dim > void solve( const Eigen::Matrix<float, Dim,1>& pInputPosition, const Eigen::Matrix<float, Dim,1>& pInputVelocity, const Eigen::Matrix<float, Dim,1>& pInputAcceleration, Eigen::Matrix<float, Dim,1>& pOutputPosition, Eigen::Matrix<float, Dim,1>& pOutputVelocity );
    
protected:
    float mTimeStep;
};
    
#pragma mark EulerSolver Implementation
    
template< unsigned int Dim >
void
EulerSolver::solve( const Eigen::Matrix<float, Dim,1>& pInputPosition, const Eigen::Matrix<float, Dim,1>& pInputVelocity, const Eigen::Matrix<float, Dim,1>& pInputAcceleration, Eigen::Matrix<float, Dim,1>& pOutputPosition, Eigen::Matrix<float, Dim,1>& pOutputVelocity )
{
    pOutputVelocity = pInputVelocity + pInputAcceleration * mTimeStep;
    pOutputPosition = pInputPosition + pInputVelocity * mTimeStep;
}

};
    
};