/** \file dab_spring_spring.h
 */

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace dab
{
    
    namespace spring
    {
        
        template< unsigned int Dim > class MassPoint;
        
#pragma mark Spring definition
        
        template< unsigned int Dim >
        class Spring
        {
        public:
            Spring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 );
            Spring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, float pDamping );
            Spring( const Spring<Dim>& pSpring );
            ~Spring();
            
            const Spring<Dim>& operator= ( const Spring<Dim>& pSpring );
            
            inline const MassPoint<Dim>* massPoint1() const;
            inline const MassPoint<Dim>* massPoint2() const;
            inline MassPoint<Dim>* massPoint1();
            inline MassPoint<Dim>* massPoint2();
            Spring<Dim>* firstPrevSpring() const;
            
            void setMassPoint1( MassPoint<Dim>* pMassPoint1 );
            void setMassPoint2( MassPoint<Dim>* pMassPoint2 );
            void setMassPoints( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 );
            inline float length() const;
            inline float restLength() const;
            inline void setRestLength( float pRestLength );
            inline const Eigen::Matrix<float, Dim, 1>& direction() const;
            inline float stiffness() const;
            inline void setStiffness( float pStiffness );
            inline float damping() const;
            inline void setDamping( float pDamping );
            
            virtual void update();
            
            operator std::string() const;
            
            std::string info(int pPropagationLevel = 0) const;
            
            friend std::ostream& operator << ( std::ostream& pOstream, const Spring<Dim>& pSpring )
            {
                pOstream << pSpring.info();
                
                return pOstream;
            };
            
        protected:
            float mLength;
            float mRestLength;
            Eigen::Matrix<float, Dim, 1> mDirection;
            float mStiffness;
            float mDamping;
            MassPoint<Dim>* mMassPoint1;
            MassPoint<Dim>* mMassPoint2;
        };
        
        typedef Spring<1>  Spring1D;
        typedef Spring<2>  Spring2D;
        typedef Spring<3>  Spring3D;
        
#pragma mark Spring implementation
        
        template< unsigned int Dim >
        Spring<Dim>::Spring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 )
        : mLength( 1.0 )
        , mRestLength( mLength )
        , mStiffness( 0.9 )
        , mDamping( 0.9 )
        , mMassPoint1( pMassPoint1 )
        , mMassPoint2( pMassPoint2 )
        {
            mMassPoint1->mSprings.push_back( this );
            mMassPoint2->mSprings.push_back( this );
            
            update();
        }
        
        template< unsigned int Dim >
        Spring<Dim>::Spring( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2, float pRestLength, float pStiffness, float pDamping )
        : mLength( 1.0 )
        , mRestLength( pRestLength )
        , mStiffness( pStiffness )
        , mDamping( pDamping )
        , mMassPoint1( pMassPoint1 )
        , mMassPoint2( pMassPoint2 )
        {
            mMassPoint1->mSprings.push_back( this );
            mMassPoint2->mSprings.push_back( this );
            
            update();
        }
        
        template< unsigned int Dim >
        Spring<Dim>::Spring( const Spring<Dim>& pSpring )
        : mLength( pSpring.mLength )
        , mRestLength( pSpring.mRestLength )
        , mStiffness( pSpring.mStiffness )
        , mDamping( pSpring.mDamping )
        , mMassPoint1( pSpring.mMassPoint1 )
        , mMassPoint2( pSpring.mMassPoint2 )
        {
            mMassPoint1->mSprings.push_back( this );
            mMassPoint2->mSprings.push_back( this );
        }
        
        template< unsigned int Dim >
        Spring<Dim>::~Spring()
        {
//            std::cout << "delete spring " << this << " begin\n";
//            
//            std::cout << "mMassPoint1 " << mMassPoint1 << " springCount " << mMassPoint1->mSprings.size() << "\n";
//            std::cout << "mMassPoint2 " << mMassPoint2 << " springCount " << mMassPoint2->mSprings.size() << "\n";
            
            auto mp1SpringIter = std::find(mMassPoint1->mSprings.begin(), mMassPoint1->mSprings.end(), this);
            if(mp1SpringIter != mMassPoint1->mSprings.end()) mMassPoint1->mSprings.erase(mp1SpringIter);
            
            auto mp2SpringIter = std::find(mMassPoint2->mSprings.begin(), mMassPoint2->mSprings.end(), this);
            if(mp2SpringIter != mMassPoint2->mSprings.end()) mMassPoint2->mSprings.erase(mp2SpringIter);
            
//            std::cout << "mMassPoint1 " << mMassPoint1 << " springCount " << mMassPoint1->mSprings.size() << "\n";
//            std::cout << "mMassPoint2 " << mMassPoint2 << " springCount " << mMassPoint2->mSprings.size() << "\n";
            
            if( mMassPoint1->mSprings.size() == 0 ) delete mMassPoint1;
            if( mMassPoint2->mSprings.size() == 0 ) delete mMassPoint2;
            
//            std::cout << "delete spring " << this << " end\n";
        }
        
        template< unsigned int Dim >
        const Spring<Dim>&
        Spring<Dim>::operator= ( const Spring<Dim>& pSpring )
        {
            mLength = pSpring.mLength;
            mRestLength = pSpring.mRestLength;
            mStiffness = pSpring.mStiffness;
            mDamping = pSpring.mDamping;
            
            setMassPoint1( pSpring.mMassPoint1 );
            setMassPoint2( pSpring.mMassPoint2 );
        }
        
        template< unsigned int Dim >
        const MassPoint<Dim>*
        Spring<Dim>::massPoint1() const
        {
            return mMassPoint1;
        }
        
        template< unsigned int Dim >
        const MassPoint<Dim>*
        Spring<Dim>::massPoint2() const
        {
            return mMassPoint2;
        }
        
        template< unsigned int Dim >
        MassPoint<Dim>*
        Spring<Dim>::massPoint1()
        {
            return mMassPoint1;
        }
        
        template< unsigned int Dim >
        MassPoint<Dim>*
        Spring<Dim>::massPoint2()
        {
            return mMassPoint2;
        }
        
        template< unsigned int Dim >
        Spring<Dim>*
        Spring<Dim>::firstPrevSpring() const
        {
            const std::vector< Spring<Dim>* >& prevSprings = mMassPoint1->springs();
            int prevSpringCount = prevSprings.size();
            
            for(int psI=0; psI<prevSpringCount; psI++)
            {
                if( prevSprings[psI]->massPoint2() == mMassPoint1 ) return prevSprings[psI];
            }
            
            return NULL;
        }
        
        template< unsigned int Dim >
        void
        Spring<Dim>::setMassPoint1( MassPoint<Dim>* pMassPoint1 )
        {
            auto mp1SpringIter = std::find(mMassPoint1->mSprings.begin(), mMassPoint1->mSprings.end(), this);
            if(mp1SpringIter != mMassPoint1->mSprings.end()) mMassPoint1->mSprings.erase(mp1SpringIter);
            
            mMassPoint1 = pMassPoint1;
            
            mMassPoint1->mSprings.push_back( this );
            
            update();
        }
        
        template< unsigned int Dim >
        void
        Spring<Dim>::setMassPoint2( MassPoint<Dim>* pMassPoint2 )
        {
            auto mp2SpringIter = std::find(mMassPoint2->mSprings.begin(), mMassPoint2->mSprings.end(), this);
            if(mp2SpringIter != mMassPoint2->mSprings.end()) mMassPoint2->mSprings.erase(mp2SpringIter);
            
            mMassPoint2 = pMassPoint2;
            
            mMassPoint2->mSprings.push_back( this );
            
            update();
        }
        
        template< unsigned int Dim >
        void
        Spring<Dim>::setMassPoints( MassPoint<Dim>* pMassPoint1, MassPoint<Dim>* pMassPoint2 )
        {
            auto mp1SpringIter = std::find(mMassPoint1->mSprings.begin(), mMassPoint1->mSprings.end(), this);
            if(mp1SpringIter != mMassPoint1->mSprings.end()) mMassPoint1->mSprings.erase(mp1SpringIter);
            
            auto mp2SpringIter = std::find(mMassPoint2->mSprings.begin(), mMassPoint2->mSprings.end(), this);
            if(mp2SpringIter != mMassPoint2->mSprings.end()) mMassPoint2->mSprings.erase(mp2SpringIter);
            
            mMassPoint1 = pMassPoint1;
            mMassPoint2 = pMassPoint2;
            
            mMassPoint1->mSprings.push_back( this );
            mMassPoint2->mSprings.push_back( this );
            
            update();
        }
        
        template< unsigned int Dim >
        float
        Spring<Dim>::length() const
        {
            return mLength;
        }
        
        template< unsigned int Dim >
        float
        Spring<Dim>::restLength() const
        {
            return mRestLength;
        }
        
        template< unsigned int Dim >
        void 
        Spring<Dim>::setRestLength( float pRestLength )
        {
            mRestLength = pRestLength;
        }
        
        template< unsigned int Dim >
        inline
        const Eigen::Matrix<float, Dim, 1>&
        Spring<Dim>::direction() const
        {
            return mDirection;
        }
        
        template< unsigned int Dim >
        float 
        Spring<Dim>::stiffness() const
        {
            return mStiffness;
        }
        
        template< unsigned int Dim >
        void 
        Spring<Dim>::setStiffness( float pStiffness )
        {
            mStiffness = pStiffness;
        }
        
        template< unsigned int Dim >
        float 
        Spring<Dim>::damping() const
        {
            return mDamping;
        }
        
        template< unsigned int Dim >
        void 
        Spring<Dim>::setDamping( float pDamping )
        {
            mDamping = pDamping;
        }
        
        template< unsigned int Dim >
        void 
        Spring<Dim>::update()
        {
            mDirection = mMassPoint2->position() - mMassPoint1->position();
            mLength = mDirection.norm();
            mDirection.normalize();
        }
        
        template< unsigned int Dim >
        Spring<Dim>::operator std::string() const
        {
            return info(0);
        }
        
        template< unsigned int Dim >
        std::string
        Spring<Dim>::info(int pPropagationLevel) const
        {
            std::stringstream ss;
            
            ss << "Length " << mLength << "\n";
            ss << "RestLength " << mRestLength << "\n";
            ss << "Stiffness " << mStiffness << "\n";
            ss << "Damping " << mDamping << "\n";
            ss << "Direction [";
            for(int d=0; d<Dim; ++d) ss << " " << mDirection[d];
            ss << " ]\n";
            
            return ss.str();
        }
        
    };
    
};