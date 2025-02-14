#include "ofApp.h"
#include "dab_spring_mass_point.h"
#include "dab_spring_spring.h"
#include "dab_spring_simulation.h"
#include "dab_spring_solver_leapfrog.h"

//--------------------------------------------------------------
void ofApp::setup()
{
//    mMP1 = new dab::spring::MassPoint<2>( 1.0, Eigen::Matrix<float, 2, 1>( 10.0, 50.0 ) );
//    mMP2 = new dab::spring::MassPoint<2>( 1.0, Eigen::Matrix<float, 2, 1>( 100.0,30.0 ) );
    
    mMP1 = new dab::spring::MassPoint<2>( 1.0, { 10.0, 50.0 } );
    mMP2 = new dab::spring::MassPoint<2>( 1.0, { 100.0,30.0 } );
    
    mSP = new dab::spring::Spring<2>( mMP1, mMP2 );
    mSP->setRestLength(50);
    
    dab::spring::Simulation<2u>& springSim = dab::spring::Simulation<2>::get();
    springSim.addSpring(mSP);
    
    
        //std::cout << "mp1:\n" << *mp1 << "\n";
        //std::cout << "mp2:\n" << *mp2 << "\n";
        //std::cout << "sp:\n" << *sp << "\n";
}

//--------------------------------------------------------------
void ofApp::update()
{
    dab::spring::Simulation<2>& springSim = dab::spring::Simulation<2>::get();
    dab::spring::LeapFrogSolver& solver = dab::spring::LeapFrogSolver::get();
    
    solver.setTimeStep(0.1);
    
    springSim.updateLength();
    springSim.updateDamping();
    springSim.solve(solver);
    springSim.update();
    
    std::cout << *mMP1 << "\n";
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackground(255, 255, 255);
    
    const Eigen::Matrix<float, 2, 1>& mp1Pos = mMP1->position();
    const Eigen::Matrix<float, 2, 1>& mp2Pos = mMP2->position();
    
    ofSetColor(0, 0, 0);
    ofDrawCircle(mp1Pos[0], mp1Pos[1], 5.0);
    ofDrawCircle(mp2Pos[0], mp2Pos[1], 5.0);
    ofDrawLine(mp1Pos[0], mp1Pos[1], mp2Pos[0], mp2Pos[1]);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
