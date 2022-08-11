#include <iostream>
#include <cmath>
using namespace std;

/*
Vehicle landing profile:
---P1--->
        |
        P2
        |
        v
        |
        P3
        |
        v
        _
*/
class Simulator {
    // TODO: if time, create proper get/set methods
    public:
        float clockCycle; // 100ms clock cycle

        float lidarMinRange;
        float singleSampleErrorOffset;
        float multipathErrorOffset; 
        float multipathErrorDuration;

        // Phase 1: transition phase attributes
        float initVelocity;
        float finalVelocity;
        float decelValue;
        float travelAltitude; 

        // Phase 2: acceleration/hover phase attributes
        float maxDescentSpeed;  

        // Phase 3: deceleration phase attributes
        float targetTouchdownVelocity; // aiming for low touchdown impact g

        // TODO: if time, fix up creation of timeseries data
        float vehicleTimeSeries[4][18000]; // 30min @100ms sampling rate

        // Set system attributes method
        void setSystemAttributes(float a, float b, float c, float d, float e) {
                clockCycle              = a; 
                lidarMinRange           = b;
                singleSampleErrorOffset = c;
                multipathErrorOffset    = d;
                multipathErrorDuration  = e;
        }

        // Phase 1 method
        void setTransAttributes(float a, float b, float c, float d) {
            initVelocity   = a; 
            finalVelocity  = b;
            decelValue     = c;
            travelAltitude = d;
        }

        // Phase 2 method
        void setAccelAttributes(float x) {
            maxDescentSpeed = x; 
        }

        // Phase 3 method
        void setDecelAttributes(float x) {
            targetTouchdownVelocity = x;
        }
};

// Kalman filter class
class Estimator {
    public:
    // pass

    void estimateStep () {}

    void updateStep () {}
};

// Method generates simulation timeseries data 
void genSimData(Simulator testObj) {
    int index = 0;
    float t = 0.0;
    float x, y;

    float timeEndP1 = abs(testObj.finalVelocity-testObj.initVelocity) / testObj.decelValue; 
    float rem = remainder(timeEndP1, testObj.clockCycle);
    float timeGuard = (timeEndP1 - rem) + testObj.clockCycle; // wait for completion of clock cycle once phase end time is reached
    //cout << timeGuard << "\n";
    
    while(t < timeGuard) {
        index++;
        t += testObj.clockCycle;
        x = testObj.initVelocity*t - 0.5*testObj.decelValue*pow(t,2);
        cout << x << "\n";

        // Set the time stamp
        testObj.vehicleTimeSeries[0][index] = t;
        // Set the x coordinate
        testObj.vehicleTimeSeries[1][index] = x;
        // Set the y coordinate
        testObj.vehicleTimeSeries[2][index] = 0; // TODO: idealisation of landing sequence; assuming heading is 0
        // Set the z coordinate
        testObj.vehicleTimeSeries[3][index] = testObj.travelAltitude;
    }

    // Complete the 
    
};


int main() {  
  
    // Initialise the simulation object & attributes
    Simulator testData1;
    testData1.setSystemAttributes(0.1, 10.0, 1.0, 0.5, 0.25);
    testData1.setTransAttributes(30.0, 0.0, 1.0, 110.0);
    testData1.setAccelAttributes(10.0);
    testData1.setDecelAttributes(0.5);
    genSimData(testData1);

    // TODO: setup another test set/simulation (if required)

    // Generate 

    return 0;
}