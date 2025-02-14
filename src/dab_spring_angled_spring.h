/** \file dab_spring_angled_spring.h
*/

#pragma once

#include <iostream>
#include "dab_spring_mass_point.h"
#include "dab_spring_spring.h"

namespace dab
{
    
namespace spring
{
    
#pragma mark Angled Spring Definition
    
template< unsigned int Dim >
class AngledSpring : public Spring< Dim >
{
public:
    AngledSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 );
    AngledSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, float pRestAngle, float pAngleStiffness, float pDamping );
    AngledSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, float pRestAngle1, float pRestAngle2, float pAngleStiffness, float pDamping );
    AngledSpring( const AngledSpring<Dim>& pSpring );
    ~AngledSpring();
    
    inline float angle1() const;
    inline float restAngle1() const;
    inline float relAngle1() const;
    inline void setRestAngle1( float pRestAngle1 );
    
    inline float angle2() const;
    inline float restAngle2() const;
    inline float relAngle2() const;
    inline void setRestAngle2( float pRestAngle2 );
    
    inline float angleStiffness() const;
    inline void setAngleStiffness( float pAngleStiffness );
    
    operator std::string() const;
    
    std::string info(int pPropagationLevel = 0) const;
    
 	friend std::ostream& operator << ( std::ostream& pOstream, const AngledSpring<Dim>& pSpring )
    {
		pOstream << pSpring.info();
        
        return pOstream;
    };
    
protected:
    float mRestAngle1;
    float mRestAngle2;
    float mAngleStiffness;
};	

typedef AngledSpring<1>  AngledSpring1D;
typedef AngledSpring<2>  AngledSpring2D;
typedef AngledSpring<3>  AngledSpring3D;

#pragma mark Angled Spring Implementation
    
template< unsigned int Dim >
AngledSpring<Dim>::AngledSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 )
: Spring<Dim>( pMassPoint1, pMassPoint2 )
, mRestAngle1( 0.0 )
, mRestAngle2( 0.0 )
, mAngleStiffness( 0.0 )
{}

template< unsigned int Dim >
AngledSpring<Dim>::AngledSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, float pRestAngle1, float pAngleStiffness, float pDamping )
: Spring<Dim>( pMassPoint1, pMassPoint2, pRestLength, pStiffness, pDamping )
, mRestAngle1( pRestAngle1 )
, mRestAngle2( 0.0 )
, mAngleStiffness( pAngleStiffness )
{}

template< unsigned int Dim >
AngledSpring<Dim>::AngledSpring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, float pRestAngle1, float pRestAngle2, float pAngleStiffness, float pDamping )
: Spring<Dim>( pMassPoint1, pMassPoint2, pRestLength, pStiffness, pDamping )
, mRestAngle1( pRestAngle1 )
, mRestAngle2( pRestAngle2 )
, mAngleStiffness( pAngleStiffness )
{}

template< unsigned int Dim >
AngledSpring<Dim>::AngledSpring( const AngledSpring<Dim>& pSpring )
: Spring<Dim>( pSpring )
, mRestAngle1( pSpring.mRestAngle1 )
, mRestAngle2( pSpring.mRestAngle2 )
, mAngleStiffness( pSpring.mAngleStiffness )
{}

template< unsigned int Dim >
AngledSpring<Dim>::~AngledSpring()
{}

template< unsigned int Dim >
inline
float
AngledSpring<Dim>::angle1() const
{
    Eigen::Matrix<float, Dim, 1> springDir = Spring<Dim>::mMassPoint2->position() - Spring<Dim>::mMassPoint1->position();
    return atan2( springDir[1], springDir[0] );
}

template< unsigned int Dim >
inline
float
AngledSpring<Dim>::restAngle1() const
{
    return mRestAngle1;
}

template< unsigned int Dim >
inline
float
AngledSpring<Dim>::relAngle1() const
{
    return angle1() - mRestAngle1;
}

template< unsigned int Dim >
inline
void
AngledSpring<Dim>::setRestAngle1( float pRestAngle1 )
{
    mRestAngle1 = pRestAngle1;
}

template< >
inline
float
AngledSpring<3>::angle2() const
{
    Eigen::Matrix<float, 3, 1> springDir = Spring<3>::mMassPoint2->position() - Spring<3>::mMassPoint1->position();
    return acos( springDir[2] / springDir.norm() );
}

template< unsigned int Dim >
inline
float
AngledSpring<Dim>::relAngle2() const
{
    return angle2() - mRestAngle2;
}

template< unsigned int Dim >
inline
float
AngledSpring<Dim>::angle2() const
{
    return 0.0;
}

template< unsigned int Dim >
inline
float
AngledSpring<Dim>::restAngle2() const
{
    return mRestAngle2;
}

template< unsigned int Dim >
inline
void 
AngledSpring<Dim>::setRestAngle2( float pRestAngle2 )
{
    mRestAngle2 = pRestAngle2;
}

template< unsigned int Dim >
inline
float 
AngledSpring<Dim>::angleStiffness() const
{
    return mAngleStiffness;
}

template< unsigned int Dim >
inline
void 
AngledSpring<Dim>::setAngleStiffness( float pAngleStiffness )
{
    mAngleStiffness = pAngleStiffness;
}

template< unsigned int Dim >
AngledSpring<Dim>::operator std::string() const
{
    return info(0);
}

template< unsigned int Dim >
std::string
AngledSpring<Dim>::info(int pPropagationLevel) const
{
    std::stringstream ss;
    
    ss << Spring<Dim>::info(pPropagationLevel);
    
    ss << "RestAngle1 " << mRestAngle1 << "\n";
    ss << "RestAngle2 " << mRestAngle2 << "\n";
    ss << "AngleStiffness " << mAngleStiffness << "\n";

    return ss.str();
}

};
    
};