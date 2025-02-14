#pragma once

#include "ofMain.h"
#include "dab_spring_mass_point.h"
#include "dab_spring_spring.h"
#include "dab_spring_simulation.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    dab::spring::MassPoint<2>* mMP1;
    dab::spring::MassPoint<2>* mMP2;
    dab::spring::Spring<2>* mSP;
    
    
};
