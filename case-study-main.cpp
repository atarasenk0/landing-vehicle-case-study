#include <cmath>
#include <iostream>

using namespace std;
#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communikation with Gnuplot

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
        |
        P4
        |
        v
        _
*/
class Simulator {
    // TODO: if time, create proper get/set methods
    public:
        float clockCycle; // 100ms clock cycle

        // System attributes
        float lidarMinRange;
        float singleSampleErrorOffset;
        float multipathErrorOffset; 
        float multipathErrorDuration;

        // Phase 1: transition phase attributes
        float transInitVelocity;
        float transFinalVelocity;
        float transDecel;
        float transCruiseAltitude; 

        // Phase 2: acceleration/hover phase attributes
        float hoverAccel; 
        float hoverInitVelocity;
        float hoverFinalVelocity;  

        // Phase 3: deceleration/touchdown phase attributes
        float descentTargetAltitude; 
        float descentFinalVelocity; // aiming for low touchdown impact g

        // TODO: if time, fix up creation of telemetry data
        float vehicleTelemetry[4][18000]; // 30min @100ms sampling rate

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
            transInitVelocity   = a; 
            transFinalVelocity  = b;
            transDecel          = c;
            transCruiseAltitude = d;
        }

        // Phase 2 method
        void setAccelAttributes(float x, float y, float z) {
            hoverInitVelocity  = x;
            hoverFinalVelocity = y;
            hoverAccel         = z; 
        }

        // Phase 3
        // Constant velocity; no attributes

        // Phase 4 method
        // Assuming that vehicle will have decelerated to final velocity at z ~= 0.0
        void setDecelAttributes(float x, float y) {
            descentTargetAltitude = x;
            descentFinalVelocity  = y;
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
    float dt = testObj.clockCycle;
    float x = 0.0, y = 0.0, z = 0.0;
    float x_K1 = 0.0, y_K1 = 0.0, z_K1 = testObj.transCruiseAltitude;
    float vx = 0.0, vy = 0.0, vz = 0.0;
    float vx_K1 = testObj.transInitVelocity, vy_K1 = 0.0, vz_K1 = 0.0;

    // ---------------------
    // In this section, calculate the guards/time limits for each landing phase
    // ---------------------
    // t = (v-u)/a
    float timeGuardP1   = abs((testObj.transFinalVelocity-testObj.transInitVelocity) / testObj.transDecel);
    //float remP1       = remainder(timeEndP1, testObj.clockCycle);
    //float timeGuardP1 = (timeEndP1 - remP1) + testObj.clockCycle; // wait for completion of clock cycle once phase end time is reached

    // t = (v-u)/a
    // s = ut + 0.5*at^2
    float timeGuardP2   = abs((testObj.hoverFinalVelocity-testObj.hoverInitVelocity) / testObj.hoverAccel); 
    float totalDistP2 = testObj.hoverInitVelocity*timeGuardP2 + 0.5*testObj.hoverAccel*pow(timeGuardP2,2);

    // t = (total-s2-s1)/(0.5*(u+v))
    float timeGuardP3   = (testObj.transCruiseAltitude - totalDistP2 - testObj.descentTargetAltitude)/(testObj.hoverFinalVelocity);

    // t = 2s/(u+v)
    float timeGuardP4   = 2.0*testObj.descentTargetAltitude/(testObj.hoverFinalVelocity + testObj.descentFinalVelocity);
    float descentDecel = (testObj.descentFinalVelocity - testObj.hoverFinalVelocity)/timeGuardP4;

    // This is the total duration of the landing phase
    float totalTimeGuard = timeGuardP1 + timeGuardP2 + timeGuardP3 + timeGuardP4;

    // ---------------------
    // In this section, establish the landing telemetry
    // ---------------------
    while (t < totalTimeGuard) {
        index++;
        t += testObj.clockCycle;

        if (t < timeGuardP1) {
            x  = vx_K1*dt + 0.5*testObj.transDecel*pow(dt,2);
            vx = vx_K1 + testObj.transDecel*dt;
            //cout << "PHASE 1: \n";
        }
        else if (t < (timeGuardP1 + timeGuardP2)) {
            x  = 0.0;
            z  = vz_K1*dt + 0.5*testObj.hoverAccel*pow(dt,2);
            vz = vz_K1 + testObj.hoverAccel*dt;
            //cout << "PHASE 2: \n";
        }
        else if (t < (timeGuardP1 + timeGuardP2 + timeGuardP3)) {
            x  = 0.0;
            z  = vz_K1*dt;
            vz = vz_K1;
            //cout << "PHASE 3: \n";
        }
        else if (t < (timeGuardP1 + timeGuardP2 + timeGuardP3 + timeGuardP4)) {
            x  = 0.0;
            z  = vz_K1*dt + 0.5*descentDecel*pow(dt,2);
            vz = vz_K1 + descentDecel*dt;
            //cout << "PHASE 4: \n";
        }
        else {
            x = y = z = 0.0;
            // pass
        }
        
        // Set the time stamp
        testObj.vehicleTelemetry[0][index] = t;
        // Set the x coordinate
        testObj.vehicleTelemetry[1][index] = x_K1 + x;
        x_K1  = x_K1 + x; // store previous value of x
        vx_K1 = vx;
        // Set the y coordinate
        testObj.vehicleTelemetry[2][index] = y_K1 + y; // TODO: idealisation of landing sequence; assuming heading is 0
        y_K1  = y_K1 + y; // store previous value of y
        vy_K1 = vy;
        // Set the z coordinate
        testObj.vehicleTelemetry[3][index] = z_K1 - z;
        z_K1  = z_K1 - z; // store previous value of z
        vz_K1 = vz;
        //cout << z_K1 << "\n";
    }
    cout << "VEHICLE HAS LANDED" << "\n";
};


int main() {  
  
    // Initialise the simulation object & attributes
    Simulator testData1;
    // setSystemAttributes(clockCycle, lidarMinRange, singleSampleErrorOffset, multipathErrorOffset, multipathErrorDuration);
    testData1.setSystemAttributes(0.1, 10.0, 1.0, 0.5, 0.25);
    // setTransAttributes(transInitVelocity, transFinalVelocity, transDecelValue, transCruiseAltitude)
    testData1.setTransAttributes(30.0, 0.0, -1.0, 1000.0);
    // setAccelAttributes(hoverInitVelocity, hoverFinalVelocity, hoverAccel)
    testData1.setAccelAttributes(0.0, 10.0, 2.0);
    // setDecelAttributes(descentTargetAltitude, descentFinalVelocity)
    testData1.setDecelAttributes(50.0, 0.5);
    genSimData(testData1);

    // Generate a plot of the above telemetry data
    try {
        Gnuplot g1("Vehicle Position");

        vector<float> t, x, y, z;

        for (int i = 0; i < 18000; i++) {
            t.push_back(testData1.vehicleTelemetry[0][i]);
            x.push_back(testData1.vehicleTelemetry[1][i]);
            y.push_back(testData1.vehicleTelemetry[2][i]);
            z.push_back(testData1.vehicleTelemetry[3][i]);
        }

        g1.savetops("testData1_output");
        g1.set_xlabel("x").set_ylabel("y").set_zlabel("z");
        g1.set_grid().set_xautoscale().set_yautoscale().set_zautoscale();
        g1.set_title("Simulator Generated Vehicle Data\\n testData1");

        g1.set_style("points").plot_xyz(x, y, z, "vehicle trajectory data");
    }
    catch (GnuplotException ge) {
        cout << ge.what() << endl;
    }
        
    return 0;
}