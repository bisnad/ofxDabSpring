/** \file dab_spring_solver_euler.cpp
*/

#include "dab_spring_solver_euler.h"

using namespace dab;
using namespace dab::spring;

#pragma mark EulerSolver implementation

EulerSolver::EulerSolver()
: mTimeStep(0.1)
{}

EulerSolver::~EulerSolver()
{}

void
EulerSolver::setTimeStep( float pTimeStep )
{
	mTimeStep = pTimeStep;
}
