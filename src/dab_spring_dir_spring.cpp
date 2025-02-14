/** \file dab_spring_dir_spring.cpp
*/

#include "dab_spring_dir_spring.h"

using namespace dab;
using namespace dab::spring;

template<>
void
DirSpring<3>::update()
{
	Spring<3>::update();

	Eigen::Matrix<float, 3, 1> rotCol1;
	Eigen::Matrix<float, 3, 1> rotCol2;
	Eigen::Matrix<float, 3, 1>  rotCol3;

	Spring<3>* prevSpring = Spring<3>::firstPrevSpring();

	Eigen::Matrix<float, 3, 1> worldPrevSpringDir;

	if( prevSpring != NULL ) worldPrevSpringDir = prevSpring->direction().normalized();
	else worldPrevSpringDir = Eigen::Matrix<float, 3, 1>(1.0, 0.0, 0.0);

	//std::cout << "spring " << sI << " wsd " << worldSpringDir << " wpsd " << worldPrevSpringDir << " lsrd " <<localSpringRestDir << "\n";

	// rotation from worldRefDir into worldPrevSpringDir

	Eigen::Quaternion<float> worldRefDir2WorldPrevSpringDirQuat = Eigen::Quaternion<float>::FromTwoVectors( sRefDir, worldPrevSpringDir );
	worldRefDir2WorldPrevSpringDirQuat.normalize();

	mWorldRestDir = worldRefDir2WorldPrevSpringDirQuat * mRestDir;
	mWorldRestDir.normalize();

	mLocalDir = worldRefDir2WorldPrevSpringDirQuat.inverse() * Spring<3>::mDirection;

	//std::cout << "localDir " << mLocalDir << " worldDir " << Spring<3>::mDirection << "\n";
	//std::cout << "localRestDir " << mRestDir << " worldRestDir " << mWorldRestDir << "\n";
}