/** \file dab_spring_dir_spring.h"
*/

#pragma once

#include <iostream>
#include "dab_spring_mass_point.h"
#include "dab_spring_spring.h"

namespace dab
{

namespace spring
{
    
#pragma mark Directional Spring Definition
    
template< unsigned int Dim >
class DirSpring : public Spring<Dim>
{
public:
	DirSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 );
	DirSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, const Eigen::Matrix<float, Dim, 1>& pRestDir, float pDirStiffness, float pDamping );
	DirSpring( const DirSpring<Dim>& pSpring );
	~DirSpring();
 
	const Eigen::Matrix<float, Dim, 1>& restDir() const;
	void setRestDir( const Eigen::Matrix<float, Dim, 1>& pDir );
 
	const Eigen::Matrix<float, Dim, 1>& worldRestDir() const;
	const Eigen::Matrix<float, Dim, 1>& localDir() const;
  
	inline float dirStiffness() const;
	inline void setDirStiffness( float pDirStiffness );
 
	void update();
    
    operator std::string() const;
    
    std::string info(int pPropagationLevel = 0) const;
    
 	friend std::ostream& operator << ( std::ostream& pOstream, const DirSpring<Dim>& pSpring )
    {
		pOstream << pSpring.info();
        
        return pOstream;
    };
    
    const static Eigen::Matrix<float, Dim, 1> sRefDir;
protected:
    
    Eigen::Matrix<float, Dim, 1> mRestDir;
    float mDirStiffness;
    
    Eigen::Matrix<float, Dim, 1>mWorldRestDir;
    Eigen::Matrix<float, Dim, 1> mLocalDir;
    
    Eigen::Matrix<float, Dim, Dim> mRefRotMatrix;
    Eigen::Matrix<float, Dim, Dim> mRefRotMatrixT;
};

typedef DirSpring<2>  DirSpring2D;
typedef DirSpring<3>  DirSpring3D;

#pragma mark Directional Spring Implementation
    
template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1> DirSpring<Dim>::sRefDir = Eigen::Matrix<float, Dim, 1>( 1.0, 0.0, 0.0 );

template< unsigned int Dim >
DirSpring<Dim>::DirSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 )
: Spring<Dim>(pMassPoint1, pMassPoint2)
, mRestDir( 1.0, 0.0, 0.0 )
, mDirStiffness( 0.9 )
{}

template< unsigned int Dim >
DirSpring<Dim>::DirSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, const Eigen::Matrix<float, Dim, 1>& pRestDir, float pDirStiffness, float pDamping )
: Spring<Dim>( pMassPoint1, pMassPoint2, pRestLength, pStiffness, pDamping )
, mRestDir( pRestDir )
, mDirStiffness(pDirStiffness)
{}

template< unsigned int Dim >
DirSpring<Dim>::DirSpring( const DirSpring<Dim>& pSpring )
{}

template< unsigned int Dim >
DirSpring<Dim>::~DirSpring()
{}

template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
DirSpring<Dim>::restDir() const
{
    return mRestDir;
}

template< unsigned int Dim >
void
DirSpring<Dim>::setRestDir( const Eigen::Matrix<float, Dim, 1>& pDir )
{
    //if( mRestDir != pDir.normalised() ) std::cout << this << " old rest dir " << mRestDir << " new rest dir " << pDir.normalised() << "\n";
    
    mRestDir = pDir;
    mRestDir.normalize();
    
    update();
}

template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
DirSpring<Dim>::worldRestDir() const
{
    return mWorldRestDir;
}

template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
DirSpring<Dim>::localDir() const
{
    return mLocalDir;
}

template< unsigned int Dim >
float
DirSpring<Dim>::dirStiffness() const
{
    return mDirStiffness;
}

template< unsigned int Dim >
void
DirSpring<Dim>::setDirStiffness( float pDirStiffness )
{
    mDirStiffness = pDirStiffness;
}

template< unsigned int Dim >
void
DirSpring<Dim>::update()
{
	// TODO: support other dimensions than 3
}

template<>
void
DirSpring<3>::update();

template< unsigned int Dim >
DirSpring<Dim>::operator std::string() const
{
    return info(0);
}

template< unsigned int Dim >
std::string
DirSpring<Dim>::info(int pPropagationLevel) const
{
    std::stringstream ss;
    
    ss << Spring<Dim>::info(pPropagationLevel);
    
    ss << "RestDir [";
    for(int d=0; d<Dim; ++d) ss << " " << mRestDir[d];
    ss << " ]\n";
    ss << "DirStiffness " << mDirStiffness << "\n";
    
    return ss.str();
}


};
    
};