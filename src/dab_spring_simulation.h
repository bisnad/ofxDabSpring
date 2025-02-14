/** \file dab_spring_simulation.h
*/

#pragma once

#include <iostream>
#include <vector>
#include <map>
#include "dab_spring_mass_point.h"
#include "dab_spring_spring.h"
#include "dab_spring_angled_spring.h"
#include "dab_spring_dir_spring.h"
#include "dab_singleton.h"

namespace dab
{

namespace spring
{
    
#pragma mark Simulation Definition
    
template< unsigned int Dim >
class Simulation : public Singleton< Simulation<Dim> >
{
public:
    Simulation();
    ~Simulation();
    
    const std::vector< MassPoint<Dim>* >& massPoints() const;
    std::vector< MassPoint<Dim>* >& massPoints();
    const std::vector< Spring<Dim>* >& springs() const;
    std::vector< Spring<Dim>* >& springs();
    const std::vector< DirSpring<Dim>* >& dirSprings() const;
    std::vector< DirSpring<Dim>* >& dirSprings();
    
    void addSpring( Spring<Dim>* pSpring );
    void addSpring( AngledSpring<Dim>* pSpring );
    void addSpring( DirSpring<Dim>* pSpring );
    void removeSpring( Spring<Dim>* pSpring );
    void removeSpring( AngledSpring<Dim>* pSpring );
    void removeSpring( DirSpring<Dim>* pSpring );
    
    void addMassPoint( MassPoint<Dim>* pMassPoint );
    void removeMassPoint( MassPoint<Dim>* pMassPoint );
    
    void addExternalForce( MassPoint<Dim>* pMassPoint, const Eigen::Matrix<float, Dim, 1>& pVector );
    void resetExternalForces();
    
    const Eigen::Matrix<float, Dim, 1>& gravity() const;
    float damping() const;
    float viscosityScale() const;
    float propulsionScale() const;
    void setGravity( const Eigen::Matrix<float, Dim, 1>& pGravity );
    void setDamping( float pVelocityDamping );
    void setViscosityScale( float pViscosityScale );
    void setPropulsionScale( float pPropulsionScale );
    
    void updateLength();
    void updateAngle();
	void updateDir();
    void updateGravity();
    void updateDamping();
    //void updatePropulsion();
    
    template<class Solver> void solve( Solver& pSolver );
    void update();
    void clear();
    
protected:
    std::vector< MassPoint<Dim>* > mMassPoints;
    std::vector< Spring<Dim>* > mSprings;
    std::vector< AngledSpring<Dim>* > mAngledSprings;
    std::vector< DirSpring<Dim>* > mDirSprings;
    unsigned long mSimStep;
    
    Eigen::Matrix<float, Dim, 1> mGravity;
    Eigen::Matrix<float, Dim, 1> windForce;
    Eigen::Matrix<float, Dim, 1> windForceLimit;
    float mViscosityScale;
    float mPropulsionScale;
    
    float mDamping;
   
    std::map< MassPoint<Dim>*, Eigen::Matrix<float, Dim, 1> > mExternalForces;
    
    bool checkMassPointInSpring( MassPoint<Dim>* pMassPoint ) const;
};

typedef Simulation<1>  Simulation1D;
typedef Simulation<2>  Simulation2D;
typedef Simulation<3>  Simulation3D;

#pragma mark Simulation Implementation
    
template< unsigned int Dim >
Simulation<Dim>::Simulation()
: mSimStep(0)
, mGravity( Eigen::Matrix<float, Dim, 1>::Constant(0.0) )
, mDamping( 0.1 )
, windForce( Eigen::Matrix<float, Dim, 1>::Constant(0.0))
, windForceLimit( Eigen::Matrix<float, Dim, 1>::Constant(0.00005) )
, mViscosityScale( 0.02 )
, mPropulsionScale( 0.02 )
{}

template< unsigned int Dim >
Simulation<Dim>::~Simulation()
{
    clear();
}

template< unsigned int Dim >
const std::vector< MassPoint<Dim>* >&
Simulation<Dim>::massPoints() const
{
    return mMassPoints;
}

template< unsigned int Dim >
std::vector< MassPoint<Dim>* >&
Simulation<Dim>::massPoints()
{
    return mMassPoints;
}
    
template< unsigned int Dim >
const std::vector< Spring<Dim>* >&
Simulation<Dim>::springs() const
{
    return mSprings;
}
    
template< unsigned int Dim >
std::vector< Spring<Dim>* >&
Simulation<Dim>::springs()
{
    return mSprings;
}
    
template< unsigned int Dim >
const std::vector< DirSpring<Dim>* >&
Simulation<Dim>::dirSprings() const
{
    return mDirSprings;
}
    
template< unsigned int Dim >
std::vector< DirSpring<Dim>* >&
Simulation<Dim>::dirSprings()
{
    return mDirSprings;
}

template< unsigned int Dim >
void
Simulation<Dim>::addSpring( Spring<Dim>* pSpring )
{
    mSprings.push_back( pSpring );
    
    MassPoint<Dim>* mp1 = pSpring->massPoint1();
    MassPoint<Dim>* mp2 = pSpring->massPoint2();
    
    addMassPoint(mp1);
    addMassPoint(mp2);
}
    
template< unsigned int Dim >
void
Simulation<Dim>::addSpring( AngledSpring<Dim>* pSpring )
{
    mSprings.push_back( pSpring );
    mAngledSprings.push_back( pSpring );
    
    MassPoint<Dim>* mp1 = pSpring->massPoint1();
    MassPoint<Dim>* mp2 = pSpring->massPoint2();
    
    addMassPoint(mp1);
    addMassPoint(mp2);
}
    
template< unsigned int Dim >
void
Simulation<Dim>::addSpring( DirSpring<Dim>* pSpring )
{
    mSprings.push_back( pSpring );
    mDirSprings.push_back( pSpring );
    
    MassPoint<Dim>* mp1 = pSpring->massPoint1();
    MassPoint<Dim>* mp2 = pSpring->massPoint2();
    
    addMassPoint(mp1);
    addMassPoint(mp2);
}
    
template< unsigned int Dim >
void
Simulation<Dim>::removeSpring( Spring<Dim>* pSpring )
{
    auto springIter = std::find(mSprings.begin(), mSprings.end(), pSpring);
    if(springIter != mSprings.end()) mSprings.erase(springIter);
    
    if( checkMassPointInSpring( pSpring->massPoint1() ) == false  ) removeMassPoint( pSpring->massPoint1() );
    if( checkMassPointInSpring( pSpring->massPoint2() ) == false  ) removeMassPoint( pSpring->massPoint2() );
}
    
template< unsigned int Dim >
void
Simulation<Dim>::removeSpring( AngledSpring<Dim>* pSpring )
{
    auto springIter = std::find(mSprings.begin(), mSprings.end(), pSpring);
    if(springIter != mSprings.end()) mSprings.erase(springIter);
    
    auto angledSpringIter = std::find(mAngledSprings.begin(), mAngledSprings.end(), pSpring);
    if(angledSpringIter != mAngledSprings.end()) mAngledSprings.erase(angledSpringIter);
    
    if( checkMassPointInSpring( pSpring->massPoint1() ) == false  ) removeMassPoint( pSpring->massPoint1() );
    if( checkMassPointInSpring( pSpring->massPoint2() ) == false  ) removeMassPoint( pSpring->massPoint2() );
}
    
template< unsigned int Dim >
void
Simulation<Dim>::removeSpring( DirSpring<Dim>* pSpring )
{
    auto springIter = std::find(mSprings.begin(), mSprings.end(), pSpring);
    if(springIter != mSprings.end()) mSprings.erase(springIter);
    
    auto dirSpringIter = std::find(mDirSprings.begin(), mDirSprings.end(), pSpring);
    if(dirSpringIter != mDirSprings.end()) mDirSprings.erase(dirSpringIter);
    
    if( checkMassPointInSpring( pSpring->massPoint1() ) == false  ) removeMassPoint( pSpring->massPoint1() );
    if( checkMassPointInSpring( pSpring->massPoint2() ) == false  ) removeMassPoint( pSpring->massPoint2() );
}
    
template< unsigned int Dim >
void
Simulation<Dim>::addMassPoint( MassPoint<Dim>* pMassPoint )
{
    auto massIter = std::find(mMassPoints.begin(), mMassPoints.end(), pMassPoint );
    if( massIter == mMassPoints.end() ) mMassPoints.push_back( pMassPoint );
}
    
template< unsigned int Dim >
void
Simulation<Dim>::removeMassPoint( MassPoint<Dim>* pMassPoint )
{
    auto forceIter = std::find(mExternalForces.begin(), mExternalForces.end(), pMassPoint );
    if( forceIter != mExternalForces.end() ) mExternalForces.erase( forceIter );
    
    auto massIter = std::find(mMassPoints.begin(), mMassPoints.end(), pMassPoint );
    if( massIter != mMassPoints.end() ) mMassPoints.erase( massIter );
}
    
template< unsigned int Dim >
void
Simulation<Dim>::resetExternalForces()
{
    mExternalForces.clear();
}
    
template< unsigned int Dim >
void
Simulation<Dim>::addExternalForce( MassPoint<Dim>* pMassPoint, const Eigen::Matrix<float, Dim, 1>& pForce )
{
    auto forceIter = std::find(mExternalForces.begin(), mExternalForces.end(), pMassPoint );
    
    if(forceIter != mExternalForces.end()) *forceIter += pForce;
    else mExternalForces[ pMassPoint ] = pForce;
}
    
template< unsigned int Dim >
const Eigen::Matrix<float, Dim, 1>&
Simulation<Dim>::gravity() const
{
    return mGravity;
}
    
template< unsigned int Dim >
float
Simulation<Dim>::damping() const
{
    return mDamping;
}
    
template< unsigned int Dim >
float
Simulation<Dim>::viscosityScale() const
{
    return mViscosityScale;
}
    
template< unsigned int Dim >
float
Simulation<Dim>::propulsionScale() const
{
    return mPropulsionScale;
}
    
template< unsigned int Dim >
void
Simulation<Dim>::setGravity( const Eigen::Matrix<float, Dim, 1>& pGravity )
{
    mGravity = pGravity;
}
    
template< unsigned int Dim >
void
Simulation<Dim>::setDamping( float pDamping )
{
    mDamping = pDamping;
}
    
template< unsigned int Dim >
void
Simulation<Dim>::setViscosityScale( float pViscosityScale )
{
    mViscosityScale = pViscosityScale;
}
    
template< unsigned int Dim >
void
Simulation<Dim>::setPropulsionScale( float pPropulsionScale )
{
    mPropulsionScale = pPropulsionScale;
}
    
template< unsigned int Dim >
void
Simulation<Dim>::updateLength()
{
    int massCount = mMassPoints.size();
    int springCount = mSprings.size();
    
    // accumulate forces
    Eigen::Matrix<float, Dim,1> springDirection;
    Eigen::Matrix<float, Dim,1> force;
    
    Spring<Dim>* spring;
    float springStiffness;
    MassPoint<Dim>* mass1;
    MassPoint<Dim>* mass2;

    for(int sI=0; sI<springCount; ++sI)
    {
        spring = mSprings[sI];
        
        springStiffness = spring->stiffness();
        if(springStiffness == 0.0) continue;
        springDirection = spring->direction();
        
        mass1 = spring->massPoint1();
        mass2 = spring->massPoint2();
        
        force = springDirection * springStiffness * ( spring->length() - spring->restLength() );
        force += ( mass2->velocity() - mass1->velocity() ) * spring->damping();
        
        mass1->addForce( force );
        mass2->addForce( force * -1.0 );
    }
    
    //	std::cout << "Simulation<Dim>::update() end\n";
}
    
template< unsigned int Dim >
void
Simulation<Dim>::updateAngle()
{
    //int massCount = mMassPoints.size();
    //int angleSpringCount = mAngledSprings.size();
    //
    //AngledSpring<Dim>* spring;
    //MassPoint<Dim>* mass1;
    //MassPoint<Dim>* mass2;

    //// TODO
}
    
template< unsigned int Dim >
void
Simulation<Dim>::updateDir()
{
    //TODO: for other dimensions than 3
}

template<>
void
Simulation<3>::updateDir();

    
template< unsigned int Dim >
void
Simulation<Dim>::updateGravity()
{
    int massCount = mMassPoints.size();
    int springCount = mSprings.size();
    MassPoint<Dim>* mass;
    
    for(int pI=0; pI<massCount; ++pI)
    {
        mass = mMassPoints[pI];
        
        //        // debug
        //        if( mMassPoints.indexOf( mass ) == 0 ) std::cout << "Simulation<Dim>::updateGravity() mass " << mass << " add force  " << mGravity << "\n";
        //        // debug done
        
        mass->addForce( mGravity );
    }
}
    
template< unsigned int Dim >
void
Simulation<Dim>::updateDamping()
{
    int massCount = mMassPoints.size();
    int springCount = mSprings.size();
    MassPoint<Dim>* mass;
    Eigen::Matrix<float, Dim, 1> force;
    
    float damping_1 = mDamping * -1.0;
    
    for(int pI=0; pI<massCount; ++pI)
    {
        mass = mMassPoints[pI];
        force = mass->velocity() * damping_1;
        
        //        // debug
        //        if( mMassPoints.indexOf( mass ) == 0 ) std::cout << "Simulation<Dim>::updateDamping() mass " << mass << " add force  " << force << "\n";
        //        // debug done
        
        mass->addForce( force );
    }
}
    
//    template< unsigned int Dim >
//    void
//    Simulation<Dim>::updatePropulsion()
//    {
//        int springCount = mSprings.size();
//        
//        QVector< MassPoint< Dim>* > propelledMassPoints;
//        
//        for(int sI=0; sI<springCount; ++sI)
//        {
//            MassPoint3D* mass1 = mSprings[sI]->massPoint1();
//            MassPoint3D* mass2 = mSprings[sI]->massPoint2();
//            const vis2::Vector3f& dir = mSprings[sI]->direction();
//            
//            if( propelledMassPoints.contains(mass1) ) continue;
//            else propelledMassPoints.append(mass1);
//            
//            float springLength = dir.length();
//            
//            if( springLength < 0.0001 ) continue;
//            
//            vis2::Vector3f force;
//            vis2::Vector3f velDiff;
//            vis2::Vector3f velSum;
//            vis2::Vector3f normVel;
//            vis2::Vector3f normVelDiff;
//            vis2::Vector3f normVelSum;
//            vis2::Vector3f normDir;
//            vis2::Vector3f tmp;
//            vis2::Vector3f normN;
//            float dampAmount;
//            float propAmount;
//            float velLength;
//            float velDiffLength;
//            float velSumLength;
//            vis2::Vector3f propForce;
//            vis2::Vector3f dampForce;
//            
//            // mass point 1 propulsion
//            {
//                const vis2::Vector3f& vel = mass1->velocity();
//                velLength = vel.length();
//                velDiff = mass1->velocity() - mass2->velocity();
//                velDiffLength = velDiff.length();
//                if( velDiffLength < 0.0001 ) continue;
//                
//                velSum = mass1->velocity() + mass2->velocity();
//                velSumLength = velSum.length();
//                if( velSumLength < 0.0001 ) continue;
//                
//                //std::cout << "spring dir " << dir << " mass1 vel " << mass1->velocity() << " mass2 vel " << mass2->velocity() << " diff " << velDiff << "\n";
//                
//                normVel = vel.normalised();
//                normVelDiff = velDiff.normalised();
//                normVelSum = velSum.normalised();
//                normDir = dir.normalised();
//                
//                // calculate propulsion
//                tmp = normDir.cross(normVelDiff);
//                tmp.normalise();
//                normN = tmp.cross(normDir);
//                normN.normalise();
//                
//                propAmount = fabs( normVelDiff.dot( normN ) ) * velDiffLength * springLength;
//                propForce = normDir * propAmount * mPropulsionScale * -1.0;
//                
//                //std::cout << "prop tmp " << tmp << " N " << normN << " amnt " << propAmount << " force " <<  propForce << "\n";
//                
//                // calculate damping
//                tmp = normDir.cross(normVel);
//                tmp.normalise();
//                normN = tmp.cross(normDir);
//                normN.normalise();
//                
//                //            dampAmount = fabs( normVel.dot( normN ) ) * velLength * springLength;
//                //            dampForce = normDir * dampAmount * mViscosityScale;
//                
//                dampAmount = fabs( normVelSum.dot( normN ) ) * velLength * springLength;
//                dampForce = normN * dampAmount * mViscosityScale;
//                
//                dampAmount = fabs( normVel.dot( normN ) ) * velLength * springLength;
//                dampForce += normDir * dampAmount * mViscosityScale;
//                
//                
//                //std::cout << "damp tmp " << tmp << " N " << normN << " amnt " << dampAmount << " force " <<  dampForce << "\n";
//                
//                force = propForce - dampForce;
//                
//                //            // debug
//                //            if( mMassPoints.indexOf( mass1 ) == 0 ) std::cout << "Simulation<Dim>::updatePropulsion() mass1 " << mass1 << " dampForce  " << dampForce << " propForce " << propForce << "\n";
//                //            // debug done
//                
//                mass1->addForce( propForce - dampForce );
//            }
//        }
//    }
    
template< unsigned int Dim >
template<class Solver>
void
Simulation<Dim>::solve( Solver& pSolver )
{
    //std::cout << "Simulation<Dim>::solve( Solver& pSolver ) begin\n";
    
    int massCount = mMassPoints.size();
    MassPoint<Dim>* mass;
    
    // numerical integration
    for(int pI=0; pI<massCount; ++pI)
    {
        mass = mMassPoints[pI];
        
        Eigen::Matrix<float, Dim,1>& mpForce = mass->force();
        
        for(int d=0; d<Dim; ++d)
        {
            if( isnan( mpForce[d] ) ) mpForce[d] = 0.0;
        }
        
        //if(pI == 0) std::cout << "point " << pI << " mass " << mass << " pos " << mass->position() << " force  " << mass->force() << "\n";

        float mpMass = mass->mass();
        Eigen::Matrix<float, Dim, 1>& mpPosition = mass->position();
        Eigen::Matrix<float, Dim, 1>& mpVelocity = mass->velocity();
        Eigen::Matrix<float, Dim, 1>& mpBackupPosition = mass->backupPosition();
        Eigen::Matrix<float, Dim, 1>& mpBackupVelocity = mass->backupVelocity();
        
        Eigen::Matrix<float, Dim, 1> scaledForce = mpForce / mpMass;
        
        if( mpMass > 0.0 ) pSolver.solve<Dim>( mpPosition, mpVelocity, scaledForce, mpBackupPosition, mpBackupVelocity );

        
        // is nan check
        for(int d=0; d<Dim; ++d)
        {
            if( isnan( mpBackupPosition[d] ) )
            {
                mpBackupPosition[d] = mpPosition[d];
            }
            if( isnan( mpBackupVelocity[d] ) )
            {
                mpBackupVelocity[d] = mpVelocity[d];
            }
        }
    }
    
    //std::cout << "Simulation<Dim>::solve( Solver& pSolver ) end\n";
}
    
template< unsigned int Dim >
void
Simulation<Dim>::update()
{
    //    std::cout << "SpringSim gravity " << mGravity << " damping " << mDamping << " propScale " << mPropulsionScale << " visScale " << mViscosityScale << "\n";
    
    int massCount = mMassPoints.size();
    int springCount = mSprings.size();
    
    //std::cout << "massCount " << massCount << " springCount " << springCount << "\n";
    
    // refresh backups
    for(int pI=0; pI<massCount; ++pI)
    {
        mMassPoints[pI]->update();
        
        //        // debug
        //        std::cout << "mp " << pI << " : " << mMassPoints[pI];
        //        const QVector< Spring<Dim>* >& springs = mMassPoints[pI]->springs();
        //        for(int sI=0; sI<springs.size(); ++sI) std::cout << " sI " << sI << " : " << springs[sI];
        //        std::cout << "\n";
        //        // debug done
        
    }
    for(int sI=0; sI<springCount; ++sI)
    {
        mSprings[sI]->update();
        
        //        // debug
        //        std::cout << "sI " << sI << " : " << mSprings[sI] << " mp1 " << mSprings[sI]->massPoint1() << " mp2 " << mSprings[sI]->massPoint2() << "\n";
        //        // debug done
    }
    
    mSimStep++;
}
    
template< unsigned int Dim >
void
Simulation<Dim>::clear()
{
    mExternalForces.clear();
	mSprings.clear();
	mAngledSprings.clear();
	mDirSprings.clear();
	mMassPoints.clear();
}
    
template< unsigned int Dim >
bool
Simulation<Dim>::checkMassPointInSpring( MassPoint<Dim>* pMassPoint ) const
{
    int springCount = mSprings.size();
    int angledSpringCount = mAngledSprings.size();
    int dirSpringCount = mDirSprings.size();
    
    Spring<Dim>* spring;
    
    for(int sI=0;sI<springCount; ++sI)
    {
        spring = mSprings[sI];
        
        if( spring->massPoint1() == pMassPoint || spring->massPoint2() == pMassPoint ) return true;
    }
    
    for(int sI=0;sI<angledSpringCount; ++sI)
    {
        spring = mAngledSprings[sI];
        
        if( spring->massPoint1() == pMassPoint || spring->massPoint2() == pMassPoint ) return true;
    }
    
    for(int sI=0;sI<dirSpringCount; ++sI)
    {
        spring = mDirSprings[sI];
        
        if( spring->massPoint1() == pMassPoint || spring->massPoint2() == pMassPoint ) return true;
    }
    
    return false;
}

};
    
};