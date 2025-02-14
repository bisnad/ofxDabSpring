

## ofxDabSpring

**Author**:  Daniel Bisig - Coventry University, UK - [ad5041@coventry.ac.uk](ad5041@coventry.ac.uk) - Zurich University of the Arts, CH - [daniel.bisig@zhdk.ch](daniel.bisig@zhdk.ch)

**Dependencies**: [ofxDabBase](https://bitbucket.org/dbisig/ofxdabbase_011/src/master/), [ofxDabMath](https://bitbucket.org/dbisig/ofxdabmath_011/src/master/)

---

## Summary

ofxDabSpring provides functionality for simulating mass-spring damper system.  The simulation can operate in one, two, or three-dimensional space. Three different types of springs are implemented. Regular springs that possess a rest length, angular springs that possess a rest angle, and directional springs that possess a rest direction. This addon is currently under development and several features have not yet been implemented. The code is compatible with OpenFrameworks 0.11 and has been tested on Windows and MacOS. The following classes are available.

**MassPoint**: A point-like mass that possesses a position and velocity and to which forces can be added.

**Spring**: a regular spring that connects to mass points. The spring has a rest length, stiffness and damping. 

**AngledSpring**: similar to a regular spring but possesses a rest angle instead of a rest length.

**DirSpring**: similar to a regular spring but possesses a rest direction instead of a rest length.

**Simulation**: Handles all springs and mass-points and calculates forces resulting from maintaining rest lenghts, directions, or angles, damping, and stiffness. 

**EulerSolver**: numerical solver based on the Euler integration method.

**LeapFrogSolver**: numerical solver based on the Leapfrog integration method.

