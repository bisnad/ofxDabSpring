/** \file dab_spring_mass_point.h
*/

#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>
namespace dab
{

namespace spring
{
    
template< unsigned int Dim > class Spring;
    
#pragma mark MassPoint definition

template< unsigned int Dim >
class MassPoint
{
public:
	friend class Spring<Dim>;
	
	MassPoint();
	MassPoint( float pMass, const Eigen::Matrix<float, Dim, 1>& pPosition );
	MassPoint( const MassPoint<Dim>& pMassPoint );
	~MassPoint();
	
	const MassPoint<Dim>& operator= ( const MassPoint<Dim>& pMassPoint );
	
	const std::vector< Spring<Dim>* >& springs() const;
	
	inline float mass() const;
	inline void setMass( float pMass );
	
	inline const Eigen::Matrix<float, Dim, 1>& position() const;
	inline Eigen::Matrix<float, Dim, 1>& position();
	inline const Eigen::Matrix<float, Dim, 1>& backupPosition() const;
	inline Eigen::Matrix<float, Dim, 1>& backupPosition();
	inline void setPosition( const Eigen::Matrix<float, Dim, 1>& pPosition );
	
	inline const Eigen::Matrix<float, Dim, 1>& velocity() const;
	inline Eigen::Matrix<float, Dim, 1>& velocity();
	inline const Eigen::Matrix<float, Dim, 1>& backupVelocity() const;
	inline Eigen::Matrix<float, Dim, 1>& backupVelocity();
	inline void setVelocity( const Eigen::Matrix<float, Dim, 1>& pVelocity );
	
	inline const Eigen::Matrix<float, Dim, 1>& force() const;
	inline Eigen::Matrix<float, Dim, 1>& force();
	inline void setForce( const Eigen::Matrix<float, Dim, 1>& pForce );
	
	void addForce( const Eigen::Matrix<float, Dim, 1>& pForce );
	void update();
    
    operator std::string() const;
    
    std::string info(int pPropagationLevel = 0) const;
    
 	friend std::ostream& operator << ( std::ostream& pOstream, const MassPoint<Dim>& pMassPoint )
    {
		pOstream << pMassPoint.info();
        
        return pOstream;
    };
	
protected:
	float mMass;
	Eigen::Matrix<float, Dim, 1> mPosition;
	Eigen::Matrix<float, Dim, 1> mBackupPosition;
	Eigen::Matrix<float, Dim, 1> mVelocity;
	Eigen::Matrix<float, Dim, 1> mBackupVelocity;
	Eigen::Matrix<float, Dim, 1>mForce;
    std::vector< Spring<Dim>* > mSprings;
};
    
typedef MassPoint<1>  MassPoint1D;
typedef MassPoint<2>  MassPoint2D;
typedef MassPoint<3>  MassPoint3D;
    
#pragma mark MassPoint implementation

template< unsigned int Dim >
MassPoint<Dim>::MassPoint()
: mPosition( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
, mMass( 0.0 )
, mBackupPosition( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
, mVelocity( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
, mBackupVelocity( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
, mForce( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
{}
    
template< unsigned int Dim >
MassPoint<Dim>::MassPoint( float pMass, const Eigen::Matrix<float, Dim, 1>& pPosition )
    : mPosition( pPosition )
, mMass( pMass )
, mBackupPosition( pPosition )
, mVelocity( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
, mBackupVelocity( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
, mForce( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
{}
    
template< unsigned int Dim >
MassPoint<Dim>::MassPoint( const MassPoint<Dim>& pMassPoint )
: mPosition( pMassPoint.mPosition )
, mMass( pMassPoint.mMass )
, mBackupPosition( pMassPoint.mBackupPosition )
, mVelocity( pMassPoint.mVelocity )
, mBackupVelocity( pMassPoint.mBackupVelocity )
, mForce( pMassPoint.mForce )
{}
    
template< unsigned int Dim >
MassPoint<Dim>::~MassPoint()
{}
    
template< unsigned int Dim >
const MassPoint<Dim>&
MassPoint<Dim>::operator= ( const MassPoint<Dim>& pMassPoint )
{
    mMass = pMassPoint.mMass;
    mPosition = pMassPoint.mPosition;
    mBackupPosition = pMassPoint.mBackupPosition;
    mVelocity = pMassPoint.mVelocity;
    mBackupVelocity = pMassPoint.mBackupVelocity;
    mForce = pMassPoint.mForce;
    
    return *this;
}
    
template< unsigned int Dim >
const std::vector< Spring<Dim>* >&
MassPoint<Dim>::springs() const
{
    return mSprings;
}
    
template< unsigned int Dim >
float
MassPoint<Dim>::mass() const
{
    return mMass;
}
    
template< unsigned int Dim >
void
MassPoint<Dim>::setMass( float pMass )
{
    mMass = pMass;
}
    
template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::position() const
{
    return mPosition;
}
    
template< unsigned int Dim >
Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::position()
{
    return mPosition;
}
    
template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::backupPosition() const
{
    return mBackupPosition;
}
    
template< unsigned int Dim >
Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::backupPosition()
{
    return mBackupPosition;
}
    
template< unsigned int Dim >
void
MassPoint<Dim>::setPosition( const Eigen::Matrix<float, Dim, 1>& pPosition )
{
    mPosition = pPosition;
    mBackupPosition = pPosition;
}
    
template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::velocity() const
{
    return mVelocity;
}
    
template< unsigned int Dim >
Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::velocity()
{
    return mVelocity;
}
    
template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::backupVelocity() const
{
    return mBackupVelocity;
}
    
template< unsigned int Dim >
Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::backupVelocity()
{
    return mBackupVelocity;
}
    
template< unsigned int Dim >
void
MassPoint<Dim>::setVelocity( const Eigen::Matrix<float, Dim, 1>& pVelocity )
{
    mVelocity = pVelocity;
    mBackupVelocity = pVelocity;
}
    
template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::force() const
{
    return mForce;
}
    
template< unsigned int Dim >
Eigen::Matrix<float, Dim, 1>&
MassPoint<Dim>::force()
{
    return mForce;
}

template< unsigned int Dim >
void 
MassPoint<Dim>::addForce( const Eigen::Matrix<float, Dim, 1>& pForce )
{
    mForce += pForce;
}
    
template< unsigned int Dim >
void 
MassPoint<Dim>::setForce( const Eigen::Matrix<float, Dim, 1>& pForce )
{
    mForce = pForce;
}
    
template< unsigned int Dim >
void 
MassPoint<Dim>::update()
{
    mPosition = mBackupPosition;
    mVelocity = mBackupVelocity;
    mForce = Eigen::Matrix<float, Dim, 1>::Constant(0.0);
}
    
template< unsigned int Dim >
MassPoint<Dim>::operator std::string() const
{
    return info(0);
}
    
template< unsigned int Dim >
std::string
MassPoint<Dim>::info(int pPropagationLevel) const
{
    std::stringstream ss;
    
    ss << "Mass " << mMass << "\n";
    ss << "Position [";
    for(int d=0; d<Dim; ++d) ss << " " << mPosition[d];
    ss << " ]\n";
    ss << "Velocity [";
    for(int d=0; d<Dim; ++d) ss << " " << mVelocity[d];
    ss << " ]\n";
    ss << "Force [";
    for(int d=0; d<Dim; ++d) ss << " " << mForce[d];
    ss << " ]\n";

    return ss.str();
}
    
};
    
};