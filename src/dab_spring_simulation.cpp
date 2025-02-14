/** \file dab_spring_simulation.cpp
*/

#include "dab_spring_simulation.h"

using namespace dab;
using namespace dab::spring;

template<>
void
Simulation<3>::updateDir()
{
    //std::cout << "Simulation<Dim>::updateDir3() begin\n";
    
    int massCount = mMassPoints.size();
    int dirSpringCount = mDirSprings.size();
    int springCount = mSprings.size();
    
    Eigen::Vector3f refDir(0.0, 0.0, 1.0);
    Eigen::Vector3f rotCol1;
    Eigen::Vector3f rotCol2;
    Eigen::Vector3f rotCol3;
    Eigen::Matrix3f curRotMax;
    Eigen::Matrix3f prefRotMax;
    Eigen::Matrix3f curRotMaxT;
    Eigen::Matrix3f prefRotMaxT;
    
    Eigen::Vector3f worldPrevSpringDir(0.0, 0.0, 0.0);
    Eigen::Vector3f worldSpringDir(0.0, 0.0, 0.0);
    Eigen::Vector3f localSpringRestDir(0.0, 0.0, 0.0);
    Eigen::Vector3f worldSpringRestDir(0.0, 0.0, 0.0);
    
    Eigen::Vector3f worldPrevSpringVector;
    Eigen::Vector3f worldSpringVector;
    Eigen::Vector3f localSpringRestVector;
    Eigen::Vector3f worldSpringRestVector;
    
    float springLength;
    float prevSpringLength;
    float dirStiffness;
    float dirDamping;
    
    Eigen::Vector3f forceDir;
    Eigen::Vector3f force;
    
    MassPoint<3>* mass1;
    MassPoint<3>* mass2;
    DirSpring<3>* spring;
    Spring<3>* prevSpring;
    
    //std::cout << "springCount " << springCount << " dirSpringCount " << dirSpringCount << "\n";
    
    //vis2::Vector3f worldRefDir(0.0, 0.0, 1.0);
    //vis2::Vector3f worldRefDir(0.0, 1.0, 0.0);
    Eigen::Vector3f worldRefDir(1.0, 0.0, 0.0);
    float leveDirDist = 10.0;
    Eigen::Vector3f localRefLeverDir1(0.0, leveDirDist, 0.0);
    Eigen::Vector3f localRefLeverDir2(0.0, -leveDirDist, 0.0);
    Eigen::Vector3f localRefLeverDir3(0.0, 0.0, leveDirDist);
    Eigen::Vector3f localRefLeverDir4(0.0, 0.0, -leveDirDist);

	//std::cout << "dirSpringCount " << dirSpringCount << "\n";
    
    for(int sI=0; sI<dirSpringCount; ++sI)
    {
        spring = mDirSprings[sI];
        if( spring->dirStiffness() <= 0.0 ) continue;
        
        prevSpring = spring->firstPrevSpring();
        
        if( prevSpring == NULL ) continue;
        
        worldSpringDir = spring->direction().normalized();
        worldPrevSpringDir = prevSpring->direction().normalized();
        localSpringRestDir = spring->restDir().normalized();
        worldSpringRestDir = spring->worldRestDir();


        //std::cout << "spring " << sI << " wsd " << worldSpringDir << " wpsd " << worldPrevSpringDir << " lsrd " <<localSpringRestDir << " wsrd " << worldSpringRestDir << "\n";
        
        //std::cout << "wsrd " << worldSpringRestDir << "\n";
        
        // rotation from worldRefDir into worldSpringDir
        Eigen::Quaternionf worldRefDir2SpringDirQuat = Eigen::Quaternionf::FromTwoVectors( worldRefDir, worldSpringDir );
        // rotation from worldRefDir into worldSpringRestDir
        Eigen::Quaternionf worldRefDir2PrefSpringDirQuat = Eigen::Quaternionf::FromTwoVectors( worldRefDir, worldSpringRestDir );
        
        worldRefDir2SpringDirQuat.normalize();
        worldRefDir2PrefSpringDirQuat.normalize();
        
        springLength = spring->length();
        prevSpringLength = prevSpring->length();
        dirStiffness = spring->dirStiffness();
        dirDamping = spring->damping();
        mass1 = spring->massPoint1();
        mass2 = spring->massPoint2();
        
        MassPoint3D* tipMass = spring->massPoint2();
        MassPoint3D* hingeMass = spring->massPoint1();
        MassPoint3D* rootMass = prevSpring->massPoint1();
        
        Eigen::Vector3f tipPos = tipMass->position();
        Eigen::Vector3f hingePos = hingeMass->position();
        Eigen::Vector3f rootPos = rootMass->position();
        
        // new
        worldSpringVector = tipPos - hingePos;
        worldPrevSpringVector = hingePos - rootPos;
        worldSpringRestVector = worldSpringRestDir;

        worldSpringRestVector /= worldSpringRestVector.norm();
        Eigen::Vector3f prefTipPos = hingePos + worldSpringRestVector;

        //std::cout << "worldSpringRestVector " << worldSpringRestVector << "/n";
        
        // new done
        
        Eigen::Vector3f worldPrefHingeLeverDir1 = worldRefDir2PrefSpringDirQuat * localRefLeverDir1;
        Eigen::Vector3f worldPrefHingeLeverDir2 = worldRefDir2PrefSpringDirQuat * localRefLeverDir2;
        Eigen::Vector3f worldPrefHingeLeverDir3 = worldRefDir2PrefSpringDirQuat * localRefLeverDir3;
        Eigen::Vector3f worldPrefHingeLeverDir4 = worldRefDir2PrefSpringDirQuat * localRefLeverDir4;
        
        worldPrefHingeLeverDir1.normalize();
        worldPrefHingeLeverDir2.normalize();
        worldPrefHingeLeverDir3.normalize();
        worldPrefHingeLeverDir4.normalize();
        
        //        float leverLength = 10.0;
        //
        //        worldPrefHingeLeverDir1.setLength(leverLength);
        //        worldPrefHingeLeverDir2.setLength(leverLength);
        //        worldPrefHingeLeverDir3.setLength(leverLength);
        //        worldPrefHingeLeverDir4.setLength(leverLength);
        
        //        worldPrefHingeLeverDir1.setLength(prevSpringLength);
        //        worldPrefHingeLeverDir2.setLength(prevSpringLength);
        //        worldPrefHingeLeverDir3.setLength(prevSpringLength);
        //        worldPrefHingeLeverDir4.setLength(prevSpringLength);
        
        Eigen::Vector3f prefHingeLeverPos1 = hingePos + worldPrefHingeLeverDir1;
        Eigen::Vector3f prefHingeLeverPos2 = hingePos + worldPrefHingeLeverDir2;
        Eigen::Vector3f prefHingeLeverPos3 = hingePos + worldPrefHingeLeverDir3;
        Eigen::Vector3f prefHingeLeverPos4 = hingePos + worldPrefHingeLeverDir4;
        
        Eigen::Vector3f prefHingeLever1TipDir = (prefTipPos - prefHingeLeverPos1);
        Eigen::Vector3f prefHingeLever2TipDir = (prefTipPos - prefHingeLeverPos2);
        Eigen::Vector3f prefHingeLever3TipDir = (prefTipPos - prefHingeLeverPos3);
        Eigen::Vector3f prefHingeLever4TipDir = (prefTipPos - prefHingeLeverPos4);
        
        //        vis2::Vector3f prefHingeLever1TipDir = (tipPos - prefHingeLeverPos1);
        //        vis2::Vector3f prefHingeLever2TipDir = (tipPos - prefHingeLeverPos2);
        //        vis2::Vector3f prefHingeLever3TipDir = (tipPos - prefHingeLeverPos3);
        //        vis2::Vector3f prefHingeLever4TipDir = (tipPos - prefHingeLeverPos4);
        
        float prefHingeLever1TipLength = prefHingeLever1TipDir.norm();
        float prefHingeLever2TipLength = prefHingeLever2TipDir.norm();
        float prefHingeLever3TipLength = prefHingeLever3TipDir.norm();
        float prefHingeLever4TipLength = prefHingeLever4TipDir.norm();
        
        Eigen::Vector3f worldCurHingeLeverDir1 = worldRefDir2SpringDirQuat * localRefLeverDir1;
        Eigen::Vector3f worldCurHingeLeverDir2 = worldRefDir2SpringDirQuat * localRefLeverDir2;
        Eigen::Vector3f worldCurHingeLeverDir3 = worldRefDir2SpringDirQuat * localRefLeverDir3;
        Eigen::Vector3f worldCurHingeLeverDir4 = worldRefDir2SpringDirQuat * localRefLeverDir4;
        
        worldCurHingeLeverDir1.normalize();
        worldCurHingeLeverDir2.normalize();
        worldCurHingeLeverDir3.normalize();
        worldCurHingeLeverDir4.normalize();
        
        //        worldCurHingeLeverDir1.setLength(leverLength);
        //        worldCurHingeLeverDir2.setLength(leverLength);
        //        worldCurHingeLeverDir3.setLength(leverLength);
        //        worldCurHingeLeverDir4.setLength(leverLength);
        
        //        worldCurHingeLeverDir1.setLength(prevSpringLength);
        //        worldCurHingeLeverDir2.setLength(prevSpringLength);
        //        worldCurHingeLeverDir3.setLength(prevSpringLength);
        //        worldCurHingeLeverDir4.setLength(prevSpringLength);
        
        /*
         std::cout << "lrld1 " << localRefLeverDir1 << " wcld1 " << worldCurHingeLeverDir1 << " wpld1 " << worldPrefLeverDir1 << "\n";
         std::cout << "lrld2 " << localRefLeverDir2 << " wcld2 " << worldCurHingeLeverDir2 << " wpld2 " << worldPrefLeverDir2 << "\n";
         std::cout << "lrld3 " << localRefLeverDir3 << " wcld3 " << worldCurHingeLeverDir3 << " wpld3 " << worldPrefLeverDir3 << "\n";
         std::cout << "lrld4 " << localRefLeverDir4 << " wcld4 " << worldCurHingeLeverDir4 << " wpld4 " << worldPrefLeverDir4 << "\n";
         */
        
        Eigen::Vector3f curHingeLeverPos1 = hingePos + worldCurHingeLeverDir1;
        Eigen::Vector3f curHingeLeverPos2 = hingePos + worldCurHingeLeverDir2;
        Eigen::Vector3f curHingeLeverPos3 = hingePos + worldCurHingeLeverDir3;
        Eigen::Vector3f curHingeLeverPos4 = hingePos + worldCurHingeLeverDir4;
        
        //        vis2::Vector3f curHingeLever1TipDir = (tipPos - curHingeLeverPos1);
        //        vis2::Vector3f curHingeLever2TipDir = (tipPos - curHingeLeverPos2);
        //        vis2::Vector3f curHingeLever3TipDir = (tipPos - curHingeLeverPos3);
        //        vis2::Vector3f curHingeLever4TipDir = (tipPos - curHingeLeverPos4);
        
        Eigen::Vector3f curHingeLever1TipDir = (tipPos - prefHingeLeverPos1);
        Eigen::Vector3f curHingeLever2TipDir = (tipPos - prefHingeLeverPos2);
        Eigen::Vector3f curHingeLever3TipDir = (tipPos - prefHingeLeverPos3);
        Eigen::Vector3f curHingeLever4TipDir = (tipPos - prefHingeLeverPos4);
        
        float curHingeLever1TipLength = curHingeLever1TipDir.norm();
        float curHingeLever2TipLength = curHingeLever2TipDir.norm();
        float curHingeLever3TipLength = curHingeLever3TipDir.norm();
        float curHingeLever4TipLength = curHingeLever4TipDir.norm();
        
        // new
        Eigen::Vector3f normCurHingeLever1TipDir = curHingeLever1TipDir;
        Eigen::Vector3f normCurHingeLever2TipDir = curHingeLever2TipDir;
        Eigen::Vector3f normCurHingeLever3TipDir = curHingeLever3TipDir;
        Eigen::Vector3f normCurHingeLever4TipDir = curHingeLever4TipDir;
        
        normCurHingeLever1TipDir.normalize();
        normCurHingeLever2TipDir.normalize();
        normCurHingeLever3TipDir.normalize();
        normCurHingeLever4TipDir.normalize();
        // new done
        
        //        vis2::Vector3f normPrefHingeLever1TipDir = prefHingeLever1TipDir.normalised();
        //        vis2::Vector3f normPrefHingeLever2TipDir = prefHingeLever2TipDir.normalised();
        //        vis2::Vector3f normPrefHingeLever3TipDir = prefHingeLever3TipDir.normalised();
        //        vis2::Vector3f normPrefHingeLever4TipDir = prefHingeLever4TipDir.normalised();
        
        
        float dirStiffness = spring->dirStiffness();
        float dirDamping = spring->damping();
        
        //dirStiffness = 0.01;
        
        //std::cout << "dirStiff " << dirStiffness << " dirDamp " << dirDamping << "\n";
        
        float hingeForceScale = 0.25;
        
        force = normCurHingeLever1TipDir * dirStiffness * ( prefHingeLever1TipLength - curHingeLever1TipLength ) * hingeForceScale;
        //force = normPrefHingeLever1TipDir * dirStiffness * ( curHingeLever1TipLength - prefHingeLever1TipLength ) * hingeForceScale;
        force += ( hingeMass->velocity() - tipMass->velocity() ) * dirDamping * hingeForceScale;
        
        //        // debug
        //        if( mMassPoints.indexOf( tipMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() tipMass " << tipMass << " add force  " << force << "\n";
        //        if( mMassPoints.indexOf( hingeMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() hingeMass " << hingeMass << " add force  " << force * -1.0 << "\n";
        //        // debug done
        
        tipMass->addForce( force );
        hingeMass->addForce( force * -1.0 );
        
        //force = normPrefHingeLever2TipDir * dirStiffness * ( curHingeLever2TipLength - prefHingeLever2TipLength ) * hingeForceScale;
        force = normCurHingeLever2TipDir * dirStiffness * ( prefHingeLever2TipLength - curHingeLever2TipLength ) * hingeForceScale;
        force += ( hingeMass->velocity() - tipMass->velocity() ) * dirDamping * hingeForceScale;
        
        //        // debug
        //        if( mMassPoints.indexOf( tipMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() tipMass " << tipMass << " add force  " << force << "\n";
        //        if( mMassPoints.indexOf( hingeMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() hingeMass " << hingeMass << " add force  " << force * -1.0 << "\n";
        //        // debug done
        
        tipMass->addForce( force );
        hingeMass->addForce( force * -1.0 );
        
        //force = normPrefHingeLever3TipDir * dirStiffness * ( curHingeLever3TipLength - prefHingeLever3TipLength ) * hingeForceScale;
        force = normCurHingeLever3TipDir * dirStiffness * ( prefHingeLever3TipLength - curHingeLever3TipLength ) * hingeForceScale;
        force += ( hingeMass->velocity() - tipMass->velocity() ) * dirDamping * hingeForceScale;
        
        //        // debug
        //        if( mMassPoints.indexOf( tipMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() tipMass " << tipMass << " add force  " << force << "\n";
        //        if( mMassPoints.indexOf( hingeMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() hingeMass " << hingeMass << " add force  " << force * -1.0 << "\n";
        //        // debug done
        
        tipMass->addForce( force );
        hingeMass->addForce( force * -1.0 );
        
        //force = normPrefHingeLever4TipDir * dirStiffness * ( curHingeLever4TipLength - prefHingeLever4TipLength ) * hingeForceScale;
        force = normCurHingeLever4TipDir * dirStiffness * ( prefHingeLever4TipLength - curHingeLever4TipLength ) * hingeForceScale;
        force += ( hingeMass->velocity() - tipMass->velocity() ) * dirDamping * hingeForceScale;
        
        //        // debug
        //        if( mMassPoints.indexOf( tipMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() tipMass " << tipMass << " add force  " << force << "\n";
        //        if( mMassPoints.indexOf( hingeMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() hingeMass " << hingeMass << " add force  " << force * -1.0 << "\n";
        //        // debug done
        
        tipMass->addForce( force );
        hingeMass->addForce( force * -1.0 );
        
        // new tip force
        float tipForceScale = 0.5; // 0.25
        
        Eigen::Vector3f curTipPrefTipDir = prefTipPos - tipPos;
        
        force = curTipPrefTipDir * dirStiffness * tipForceScale;
        
        //        // debug
        //        if( mMassPoints.indexOf( tipMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() tipMass " << tipMass << " add force  " << force << "\n";
        //        if( mMassPoints.indexOf( hingeMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() hingeMass " << hingeMass << " add force  " << force * -1.0 << "\n";
        //        // debug done
        
        tipMass->addForce( force );
        hingeMass->addForce( force * -1.0 );
        
        // new root force
        float rootForceScale = 0.5; // 0.25
        
        Eigen::Vector3f rootCurTipDir = tipPos - rootPos;
        Eigen::Vector3f rootPrefTipDir = prefTipPos - rootPos;
        Eigen::Vector3f normRootCurTipDir = rootCurTipDir;
        normRootCurTipDir.normalize();
        float rootCurTipLenth = rootCurTipDir.norm();
        float rootPrefTipLength = rootPrefTipDir.norm();
        
        force = normRootCurTipDir * ( rootPrefTipLength - rootCurTipLenth ) * dirDamping * rootForceScale;
        
        //        // debug
        //        if( mMassPoints.indexOf( tipMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() tipMass " << tipMass << " add force  " << force << "\n";
        //        if( mMassPoints.indexOf( hingeMass ) == 0 ) std::cout << "Simulation<Dim>::updateDir() hingeMass " << hingeMass << " add force  " << force * -1.0 << "\n";
        //        // debug done
        
        tipMass->addForce( force );
        hingeMass->addForce( force * -1.0 );

		//std::cout << "spring " << sI << " force " << force << "\n";
    }
    
    //std::cout << "Simulation<Dim>::updateDir() end\n";
}
    