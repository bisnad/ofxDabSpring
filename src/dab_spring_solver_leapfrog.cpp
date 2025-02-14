/** \file dab_spring_solver_leapfrog.cpp
*/

#include "dab_spring_solver_leapfrog.h"

using namespace dab;
using namespace dab::spring;

#pragma mark LeapFrogSolver implementation

LeapFrogSolver::LeapFrogSolver()
: mTimeStep(0.1)
{}

LeapFrogSolver::~LeapFrogSolver()
{}

void
LeapFrogSolver::setTimeStep( float pTimeStep )
{
	mTimeStep = pTimeStep;
}