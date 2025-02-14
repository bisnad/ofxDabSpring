/** \file dab_spring_solver_leapfrog.h
*/

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "dab_singleton.h"

namespace dab
{
    
namespace spring
{
    
#pragma mark LeapFrogSolver Definition
    
class LeapFrogSolver : public Singleton< LeapFrogSolver >
{
public:
    LeapFrogSolver();
    ~LeapFrogSolver();
    
    void setTimeStep( float pTimeStep );
    
    template< int Dim > void solve( const Eigen::Matrix<float, Dim,1>& pInputPosition, const Eigen::Matrix<float, Dim,1>& pInputVelocity, const Eigen::Matrix<float, Dim,1>& pInputAcceleration, Eigen::Matrix<float, Dim,1>& pOutputPosition, Eigen::Matrix<float, Dim,1>& pOutputVelocity );
    
protected:
    float mTimeStep;
};
    
#pragma mark LeapFrogSolver Implementation

template< int Dim >
void
LeapFrogSolver::solve( const Eigen::Matrix<float, Dim,1>& pInputPosition, const Eigen::Matrix<float, Dim,1>& pInputVelocity, const Eigen::Matrix<float, Dim,1>& pInputAcceleration, Eigen::Matrix<float, Dim,1>& pOutputPosition, Eigen::Matrix<float, Dim,1>& pOutputVelocity )
{
    //std::cout << "NumericalSolver::solve begin\n";

    pOutputVelocity = pInputVelocity + pInputAcceleration * mTimeStep;
    pOutputPosition = pInputPosition + pOutputVelocity * mTimeStep;
    
    //if( pOutputPosition[1] < 0.0 ) pOutputPosition[1] = 0.0;
    
    //std::cout << "ipos " << pInputPosition[0] << " " << pInputPosition[1] << " ivel " << pInputVelocity[0] << " " << pInputVelocity[1] << " iacc " << pInputAcceleration[0] << " " << pInputAcceleration[1] << " oPos " << pOutputPosition[0] << " " << pOutputPosition[1] << " oVel " << pOutputVelocity[0] << " " << pOutputVelocity[1] << "\n";
    
    //std::cout << "NumericalSolver::solve end\n";
}
    
};

};