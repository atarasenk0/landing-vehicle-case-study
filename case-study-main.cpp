#include <cmath>
#include <iostream>
#include <Eigen/Dense>   // matrix manipulation library

#include "gnuplot_i.hpp" // Gnuplot class handles POSIX-Pipe-communication with Gnuplot

using namespace std;
using Eigen::MatrixXd;

#define NSTATES 7 // time + number of vehicle states considered
#define NPOINTS 18000 // number of samples points require for 30min @10Hz sampling rate
#define PI 3.14159
#define g 9.81

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
        float clockCycle; 

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
        float transHeadingAngle;

        // Phase 2: acceleration/hover phase attributes
        float hoverAccel; 
        float hoverInitVelocity;
        float hoverFinalVelocity;  

        // Phase 3: deceleration/touchdown phase attributes
        float descentTargetAltitude; 
        float descentFinalVelocity; // aiming for low touchdown impact g

        // TODO: if time, fix up creation of telemetry data i.e dynamic implementation
        float vehicleTelemetry[NSTATES][NPOINTS];
        float predVehicleState[NSTATES][NPOINTS];

        // Set system attributes method
        void setSystemAttributes(float a, float b, float c, float d, float e) {
            clockCycle              = a; 
            lidarMinRange           = b;
            singleSampleErrorOffset = c;
            multipathErrorOffset    = d;
            multipathErrorDuration  = e;
        }

        // Phase 1 method
        void setTransAttributes(float a, float b, float c, float d, float e) {
            transInitVelocity   = a; 
            transFinalVelocity  = b;
            transDecel          = c;
            transCruiseAltitude = d;
            transHeadingAngle   = e;
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

        // Outside method declaration
        void genSimData();
};


// Method generates simulation timeseries data 
void Simulator::genSimData() {
    int index = 0;
    float t = 0.0;
    float dt = clockCycle;
    float x = 0.0, y = 0.0, z = 0.0;
    float x_K1 = 0.0, y_K1 = 0.0, z_K1 = transCruiseAltitude;
    float vx = 0.0, vy = 0.0, vz = 0.0;
    float vx_K1 = transInitVelocity, vy_K1 = 0.0, vz_K1 = 0.0;

    // ---------------------
    // In this section, calculate the guards/time limits for each landing phase
    // ---------------------
    // t = (v-u)/a
    float timeGuardP1 = abs((transFinalVelocity-transInitVelocity) / transDecel);
    //float remP1       = remainder(timeEndP1, clockCycle);
    //float timeGuardP1 = (timeEndP1 - remP1) + clockCycle; // wait for completion of clock cycle once phase end time is reached

    // t = (v-u)/a
    // s = ut + 0.5*at^2
    float timeGuardP2 = abs((hoverFinalVelocity-hoverInitVelocity) / hoverAccel); 
    float totalDistP2 = hoverInitVelocity*timeGuardP2 + 0.5*hoverAccel*pow(timeGuardP2,2);

    // t = (total-s2-s1)/(0.5*(u+v))
    float timeGuardP3 = (transCruiseAltitude - totalDistP2 - descentTargetAltitude)/(hoverFinalVelocity);

    // t = 2s/(u+v)
    float timeGuardP4  = 2.0*descentTargetAltitude/(hoverFinalVelocity + descentFinalVelocity);
    float descentDecel = (descentFinalVelocity - hoverFinalVelocity)/timeGuardP4;

    // This is the total duration of the landing phase
    float totalTimeGuard = timeGuardP1 + timeGuardP2 + timeGuardP3 + timeGuardP4;

    // ---------------------
    // In this section, establish the landing telemetry
    // Variables with _K1 addendum are used to store previous value
    // ---------------------
    while (t < totalTimeGuard) {
        index++;
        t += clockCycle;

        if (t < timeGuardP1) {
            x  = vx_K1*dt + 0.5*transDecel*pow(dt,2);
            vx = (vx_K1 + transDecel*dt);//*cos(double(PI*transHeadingAngle/180.0));

            y  = vy_K1*dt + 0.5*transDecel*pow(dt,2);
            vy = (vy_K1 + transDecel*dt);//*sin(double(PI*transHeadingAngle/180.0));
            //cout << "PHASE 1: \n";
        }
        else if (t < (timeGuardP1 + timeGuardP2)) {
            x  = 0.0;
            y  = 0.0;
            z  = vz_K1*dt + 0.5*hoverAccel*pow(dt,2);
            vz = vz_K1 + hoverAccel*dt;
            //cout << "PHASE 2: \n";
        }
        else if (t < (timeGuardP1 + timeGuardP2 + timeGuardP3)) {
            x  = 0.0;
            y  = 0.0;
            z  = vz_K1*dt;
            vz = vz_K1;
            //cout << "PHASE 3: \n";
        }
        else if (t < (timeGuardP1 + timeGuardP2 + timeGuardP3 + timeGuardP4)) {
            x  = 0.0;
            y  = 0.0;
            z  = vz_K1*dt + 0.5*descentDecel*pow(dt,2);
            vz = vz_K1 + descentDecel*dt;
            //cout << "PHASE 4: \n";
        }
        else {
            x = y = z = 0.0;
            // pass
        }
        
        // Set the time stamp
        vehicleTelemetry[0][index] = t;
        // Set the x coordinate
        vehicleTelemetry[1][index] = x_K1 + x;
        x_K1  = x_K1 + x; // store previous value of x
        vx_K1 = vx;
        // Set the y coordinate
        vehicleTelemetry[2][index] = y_K1 + y; // TODO: idealisation of landing sequence; assuming heading is 0
        y_K1  = y_K1 + y; // store previous value of y
        vy_K1 = vy;
        // Set the z coordinate
        vehicleTelemetry[3][index] = z_K1 - z;
        z_K1  = z_K1 - z; // store previous value of z
        vz_K1 = vz;
        //cout << z_K1 << "\n";
    }
    cout << "VEHICLE HAS LANDED" << "\n";
};


// Kalman filter class
// 3DoF model
class Estimator3DoF {
    public:
        MatrixXd F  = MatrixXd(6,6);
        MatrixXd G  = MatrixXd(6,3);
        MatrixXd _X = MatrixXd(6,1);

    void setStateAttributes (MatrixXd x, MatrixXd y) {
        F = x;
        G = y;
    }

    // Class evaluates the state extrapolation equation  
    MatrixXd estimateState (MatrixXd x, MatrixXd u) {
        _X = F*x + G*u;
        return _X;
    }
};


// Kalman filter class
// 6DoF model
class Estimator6DoF {
    public:
        // pass
};


// Vehicle parameter class
// https://sci-hub.se/10.1109/control.2014.6915128
// TODO; this class is potentially not required; depending on the state model
class Vehicle {
    public:
        float Jxx; 
        float Jyy; 
        float Jzz;
        float m;
    
    void setVehicleAttributes(double x, double y, double z, double a) {
        Jxx = x; 
        Jyy = y; 
        Jzz = z;
        m   = a; 
    }
};

// Function plots/saves data to a *.ps file in work directory
// TODO: once idealised example is extended, extend plotting to xyz plot inplace of xy only
// TODO: generalise function to plot required arrays only? Not sure if this is possible...
void plotTelemetryData(Simulator testData, string fileName) {
    // Generate a plot of the above telemetry data
    try {
        Gnuplot g1("Vehicle Position");

        vector<float> t, x, y, z;

        for (int i = 0; i < NPOINTS; i++) {
            t.push_back(testData.vehicleTelemetry[0][i]);
            x.push_back(testData.vehicleTelemetry[1][i]);
            y.push_back(testData.vehicleTelemetry[2][i]);
            z.push_back(testData.vehicleTelemetry[3][i]);
        }

        g1.savetops(fileName);
        g1.set_xlabel("x").set_ylabel("z");//.set_zlabel("z");
        g1.set_grid().set_xrange(0,500).set_yrange(0,1200);//set_zrange(0,1200);
        g1.set_title("Simulator Generated Vehicle Data\\n testData1");
        g1.set_style("points").plot_xy(x, z, "vehicle trajectory data");
    }
    catch (GnuplotException ge) {
        cout << ge.what() << endl;
    }
}

// Function compares simulated telemetry data with estimated
void compareVehicleData(Simulator testData, string fileName) {
    // Generate a plot of the above telemetry data
    try {
        Gnuplot g1("Vehicle Position");

        vector<float> t, x, y, z, _x, _y, _z;

        for (int i = 0; i < NPOINTS; i++) {
            //t.push_back(testData.vehicleTelemetry[0][i]);
            x.push_back(testData.vehicleTelemetry[1][i]);
            y.push_back(testData.vehicleTelemetry[2][i]);
            z.push_back(testData.vehicleTelemetry[3][i]);

            //t.push_back(testData.predVehicleState[0][j]);
            _x.push_back(testData.predVehicleState[1][i]);
            _y.push_back(testData.predVehicleState[2][i]);
            _z.push_back(testData.predVehicleState[3][i]);
        }

        g1.savetops(fileName);
        g1.set_xlabel("x").set_ylabel("z");//.set_zlabel("z");
        g1.set_grid().set_xrange(0,500).set_yrange(0,1200);//set_zrange(0,1200);
        g1.set_title("Simulator Generated Vehicle Data\\n testData1");
        g1.set_style("points").plot_xy(x, z, "Simulated vehicle trajectory").plot_xy(_x, _z, "Predicted vehicle trajectory");
    }
    catch (GnuplotException ge) {
        cout << ge.what() << endl;
    }
}


int main() {  
  
    // Initialise the simulation object & attributes
    Simulator testData1;
    // setSystemAttributes(clockCycle, lidarMinRange, singleSampleErrorOffset, multipathErrorOffset, multipathErrorDuration);
    testData1.setSystemAttributes(0.5, 10.0, 1.0, 0.5, 0.25);
    // setTransAttributes(transInitVelocity, transFinalVelocity, transDecelValue, transCruiseAltitude, transHeadingAngle)
    testData1.setTransAttributes(30.0, 0.0, -1.0, 1000.0, 30.0);
    // setAccelAttributes(hoverInitVelocity, hoverFinalVelocity, hoverAccel)
    testData1.setAccelAttributes(0.0, 10.0, 2.0);
    // setDecelAttributes(descentTargetAltitude, descentFinalVelocity)
    testData1.setDecelAttributes(50.0, 0.5);
    testData1.genSimData();

    // Plot/save telemetry data
    plotTelemetryData(testData1, "testData1_output_check");

    // Initialise the estimation object for 3DoF model
    // Considering the following example:
    // https://www.kalmanfilter.net/stateextrap.html#ex2
    // TODO: doubling up on parameter/attribute names... not a good implementation
    Estimator3DoF vehicleState3DoF;
    MatrixXd F  = MatrixXd(6,6); // state transition matrix
    MatrixXd G  = MatrixXd(6,3); // control matrix
    MatrixXd _X = MatrixXd(6,1); // extrapolation equation result
    MatrixXd x  = MatrixXd(6,1); // current vehicle state
    MatrixXd u  = MatrixXd(3,1); // control input

    F << 1.0, 0.0, 0.0, testData1.clockCycle, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, testData1.clockCycle, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, testData1.clockCycle,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    
    G << 0.5*pow(testData1.clockCycle,2), 0.0, 0.0,
         0.0, 0.5*pow(testData1.clockCycle,2), 0.0,
         0.0, 0.0, 0.5*pow(testData1.clockCycle,2),
         testData1.clockCycle, 0.0, 0.0,
         0.0, testData1.clockCycle, 0.0,
         0.0, 0.0, testData1.clockCycle;

    vehicleState3DoF.setStateAttributes(F, G);

    // Create an estimate of the vehicle state
    // Utilise the simulated data as the current state
    for (int i = 0; i < NPOINTS; i++) {
        x << testData1.vehicleTelemetry[1][i], 
             testData1.vehicleTelemetry[2][i], 
             testData1.vehicleTelemetry[3][i], 
             testData1.vehicleTelemetry[4][i], 
             testData1.vehicleTelemetry[5][i], 
             testData1.vehicleTelemetry[6][i];

        _X = vehicleState3DoF.estimateState(x, u);

        testData1.predVehicleState[1][i] = _X(0,0);
        testData1.predVehicleState[2][i] = _X(1,0);
        testData1.predVehicleState[3][i] = _X(2,0);
        testData1.predVehicleState[1][i] = _X(3,0);
        testData1.predVehicleState[2][i] = _X(4,0);
        testData1.predVehicleState[3][i] = _X(5,0);
    } 

    // Compare simulated data to predicted results
    // TODO: DRY......
    compareVehicleData(testData1, "testData1_output_compare");

    // Initialise the vehicle object & attributes
    // Parameters based on:
    // https://sci-hub.se/10.1109/control.2014.6915128
    Vehicle SwoopAeroVehicle1;
    // setVehicleAttributes(Jxx, Jyy, Jzz, m);
    SwoopAeroVehicle1.setVehicleAttributes(0.00517, 0.00517, 0.017, 0.80);

    // Initialise the estimation object(s) for 6DoF model
    Estimator6DoF vehicleState6DoF;

    return 0;
}