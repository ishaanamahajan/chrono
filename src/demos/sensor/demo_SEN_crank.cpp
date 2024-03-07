// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
//  Demo code about
//
//  - constraints and 'motor' objects
//  - using IRRLICHT as a realtime 3D viewer of a slider-crank mechanism
//    simulated with Chrono::Engine.
//  - using the real-time step.
//
// This is just a possible method of integration of Chrono::Engine + Irrlicht;
// many others are possible.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::sensor;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------
// IMU parameters
// -----------------------------------------------------------------------------

// IMU update rate in Hz
int imu_update_rate = 100;

// IMU lag (in seconds) between sensing and when data becomes accessible
float imu_lag = 0;

// IMU collection time (in seconds) of each sample
float imu_collection_time = 0;

// -----------------------------------------------------------------------------
// GPS parameters
// -----------------------------------------------------------------------------

// GPS update rate in Hz
int gps_update_rate = 10;

// GPS lag (in seconds) between sensing and when data becomes accessible
float gps_lag = 0;

// Collection time (in seconds) of eacn sample
float gps_collection_time = 0;

// Origin used as the gps reference point
// Located in Madison, WI
ChVector<> gps_reference(43.0723, -89.413, 260.0);

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 20.0f;

// Save data
bool save = true;

// Output directories
const std::string out_dir = "SENSOR_OUTPUT/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // 1- Create a Chrono physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.

    ChSystemNSC sys;

    // 2- Create the rigid bodies of the slider-crank mechanical system
    //   (a crank, a rod, a truss), maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the truss
    auto my_body_A = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_A);
    my_body_A->SetBodyFixed(true);  // truss does not move!
    my_body_A->SetName("Ground-Truss");

    auto cyl_g = chrono_types::make_shared<ChVisualShapeCylinder>(0.03, 0.1);
    my_body_A->AddVisualShape(cyl_g);

    // ..the crank
    auto crank_rot = ChQuaternion<>();
    crank_rot.Q_from_AngAxis(CH_C_PI / 4.0, ChVector<double>(0, 0, 1));
    auto crank_pos = ChVector<double>(0.5, 0.5, 0);
    auto my_body_B = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_B);
    my_body_B->SetPos(crank_pos);  // position of COG of crank
    my_body_B->SetMass(1.0);
    my_body_B->SetInertiaXX(ChVector<>(0.005, 0.1, 0.1));
    my_body_B->SetRot(crank_rot);
    my_body_B->SetName("Crank");

    auto bar1_length = sqrt(2);
    auto box_b = chrono_types::make_shared<ChVisualShapeBox>(bar1_length, 0.1, 0.1);
    my_body_B->AddVisualShape(box_b);
    my_body_B->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.2f, 0.2f));

    // ..the rod
    auto rod_rot = ChQuaternion<>();
    rod_rot.Q_from_AngAxis(CH_C_PI - (CH_C_PI / 6.0), ChVector<double>(0, 0, 1));
    auto rod_pos = ChVector<double>(1 + (sqrt(3) / 2.), 0.5, 0);
    auto my_body_C = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_C);
    my_body_C->SetPos(rod_pos);  // position of COG of rod
    my_body_C->SetRot(rod_rot);
    my_body_C->SetInertiaXX(ChVector<>(0.005, 0.5, 0.5));
    my_body_C->SetMass(3);
    my_body_C->SetName("Rod");

    auto bar2_length = 2;
    auto box_c = chrono_types::make_shared<ChVisualShapeBox>(bar2_length, 0.1, 0.1);
    my_body_C->AddVisualShape(box_c);
    my_body_C->GetVisualShape(0)->SetColor(ChColor(0.0f, 0.0f, 1.f));

    // .. The slider
    auto my_body_D = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_D);
    my_body_D->SetPos(ChVector<>(1. + sqrt(3.), 0, 0));  // position of COG of slider
    my_body_D->SetMass(1.0);
    my_body_D->SetInertiaXX(ChVector<>(1., 1., 1.));
    my_body_D->SetRot(ChQuaternion<>(1, 0, 0, 0));
    my_body_D->SetName("Slider");

    auto box_d = chrono_types::make_shared<ChVisualShapeBox>(0.1, 0.1, 0.2);
    my_body_D->AddVisualShape(box_d);
    my_body_D->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.6f));

    // 3- Create constraints: the mechanical joints between the rigid bodies.

    // revolute ground and crank
    auto ground_crank = chrono_types::make_shared<ChLinkLockRevolute>();
    ground_crank->SetName("RevJointGroundCrank");
    ground_crank->Initialize(my_body_A, my_body_B, ChCoordsys<>(ChVector<>(0, 0, 0)));
    sys.AddLink(ground_crank);

    // Revolute crank and rod
    auto crank_rod = chrono_types::make_shared<ChLinkLockRevolute>();
    crank_rod->SetName("RevJointCrankRod");
    crank_rod->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(1, 1, 0)));
    sys.AddLink(crank_rod);

    // Sperical rod and slider
    auto rod_slider = chrono_types::make_shared<ChLinkLockSpherical>();
    rod_slider->SetName("SpherJointRodSlider");
    rod_slider->Initialize(my_body_C, my_body_D, ChCoordsys<>(ChVector<>(1 + sqrt(3.), 0, 0)));
    sys.AddLink(rod_slider);

    // Translational slider and ground
    auto z2x = ChQuaternion<>();
    z2x.Q_from_AngAxis(CH_C_PI / 2.0, ChVector<double>(0, 1, 0));
    auto slider_ground = chrono_types::make_shared<ChLinkLockPrismatic>();
    slider_ground->SetName("TransJointSliderGround");
    slider_ground->Initialize(my_body_D, my_body_A, ChCoordsys<>(ChVector<>(1 + sqrt(3.), 0, 0), z2x));
    sys.AddLink(slider_ground);

    // .. a motor between crank and truss
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(my_body_A, my_body_B, ChFrame<>(ChVector<>(0, 0, 0)));
    motor->SetName("RotationalMotor");
    sys.AddLink(motor);
    auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
    motor->SetSpeedFunction(my_speed_function);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    // ---------------------------------------------
    // Create a IMU and add it to the sensor manager
    // ---------------------------------------------
    // Create the imu noise model
    std::shared_ptr<ChNoiseModel> acc_noise_model;
    std::shared_ptr<ChNoiseModel> gyro_noise_model;
    std::shared_ptr<ChNoiseModel> mag_noise_model;

    acc_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,                          //
                                                                    ChVector<double>({0., 0., 0.}),           // mean,
                                                                    ChVector<double>({0.001, 0.001, 0.001}),  // stdev,
                                                                    .0001,             // bias_drift,
                                                                    .1);               // tau_drift,
    gyro_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,  // float updateRate,
                                                                     ChVector<double>({0., 0., 0.}),  // float mean,
                                                                     ChVector<double>({0.001, 0.001, 0.001}),  // float
                                                                     .001,  // double bias_drift,
                                                                     .1);   // double tau_drift,
    mag_noise_model =
        chrono_types::make_shared<ChNoiseNormal>(ChVector<double>({0., 0., 0.}),            // float mean,
                                                 ChVector<double>({0.001, 0.001, 0.001}));  // float stdev,

    // add an accelerometer, gyroscope, and magnetometer to one of the pendulum legs
    auto imu_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(my_body_D,         // body to which the IMU is attached
                                                                imu_update_rate,   // update rate
                                                                imu_offset_pose,   // offset pose from body
                                                                acc_noise_model);  // IMU noise model

    acc->SetName("IMU - Accelerometer");
    acc->SetLag(imu_lag);
    acc->SetCollectionWindow(imu_collection_time);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
    manager->AddSensor(acc);

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(my_body_D,          // body to which the IMU is attached
                                                             imu_update_rate,    // update rate
                                                             imu_offset_pose,    // offset pose from body
                                                             gyro_noise_model);  // IMU noise model
    gyro->SetName("IMU - Accelerometer");
    gyro->SetLag(imu_lag);
    gyro->SetCollectionWindow(imu_collection_time);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
    manager->AddSensor(gyro);                                           // Add the IMU sensor to the sensor manager

    auto mag = chrono_types::make_shared<ChMagnetometerSensor>(my_body_D,        // body to which the IMU is attached
                                                               imu_update_rate,  // update rate
                                                               imu_offset_pose,  // offset pose from body
                                                               mag_noise_model,  // IMU noise model
                                                               gps_reference);
    mag->SetName("IMU - Accelerometer");
    mag->SetLag(imu_lag);
    mag->SetCollectionWindow(imu_collection_time);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());  // Add a filter to access the imu data
    manager->AddSensor(mag);                                             // Add the IMU sensor to the sensor manager

    // ---------------------------------------------
    // Create a GPS and add it to the sensor manager
    // ---------------------------------------------
    // Create the gps noise model
    std::shared_ptr<ChNoiseModel> gps_noise_model_normal;
    std::shared_ptr<ChNoiseModel> gps_noise_model_none;
    std::shared_ptr<ChNoiseModel> gps_noise_model_random_walk;

    // Set the gps noise model to a gaussian model
    gps_noise_model_normal =
        chrono_types::make_shared<ChNoiseNormal>(ChVector<double>(1.f, 1.f, 1.f),  // Mean
                                                 ChVector<double>(2.f, 3.f, 1.f)   // Standard Deviation
        );
    // Set the gps noise model to none (does not affect the data)
    gps_noise_model_none = chrono_types::make_shared<ChNoiseNone>();
    gps_noise_model_random_walk =
        chrono_types::make_shared<ChNoiseRandomWalks>(0, 0.016, 100, 0.03, 0.005, gps_reference);

    // add a GPS sensor to one of the boxes
    auto gps_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto gps_normal = chrono_types::make_shared<ChGPSSensor>(
        my_body_D,              // body to which the GPS is attached
        gps_update_rate,        // update rate
        gps_offset_pose,        // offset pose from body
        gps_reference,          // reference GPS location (GPS coordinates of simulation origin)
        gps_noise_model_normal  // noise model to use for adding GPS noise
    );
    gps_normal->SetName("GPS_NORMAL");
    gps_normal->SetLag(gps_lag);
    gps_normal->SetCollectionWindow(gps_collection_time);
    gps_normal->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps_normal);

    auto gps_none = chrono_types::make_shared<ChGPSSensor>(
        my_body_D,            // body to which the GPS is attached
        gps_update_rate,      // update rate
        gps_offset_pose,      // offset pose from body
        gps_reference,        // reference GPS location (GPS coordinates of simulation origin)
        gps_noise_model_none  // noise model to use for adding GPS noise
    );
    gps_none->SetName("GPS_NONE");
    gps_none->SetLag(gps_lag);
    gps_none->SetCollectionWindow(gps_collection_time);
    gps_none->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps_none);

    auto gps_random_walk = chrono_types::make_shared<ChGPSSensor>(
        my_body_D,                   // body to which the GPS is attached
        gps_update_rate,             // update rate
        gps_offset_pose,             // offset pose from body
        gps_reference,               // reference GPS location (GPS coordinates of simulation origin)
        gps_noise_model_random_walk  // noise model to use for adding GPS noise
    );
    gps_random_walk->SetName("GPS_RANDOM_WALK");
    gps_random_walk->SetLag(gps_lag);
    gps_random_walk->SetCollectionWindow(gps_collection_time);
    gps_random_walk->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps_random_walk);

    // 4- Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Simple slider-crank example");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, -6));
    vis->AddTypicalLights();

    // -----------------
    // Initialize output
    // -----------------

    std::string imu_file = out_dir + "imu/";
    std::string gps_file = out_dir + "gps/";

    if (!filesystem::create_directory(filesystem::path(imu_file))) {
        std::cout << "Error creating directory " << imu_file << std::endl;
        return 1;
    }

    if (!filesystem::create_directory(filesystem::path(gps_file))) {
        std::cout << "Error creating directory " << gps_file << std::endl;
        return 1;
    }

    // Create a CSV writer to record the IMU data
    imu_file += "slider_imu.csv";
    utils::CSV_writer imu_csv(" ");

    // Create a CSV writer to record the GPS data
    gps_file += "slider_gps.csv";
    utils::CSV_writer gps_csv(" ");

    // ==============================
    // REMOVE IN MAIN DEMO
    // ==============================
    // Initialize buffers
    UserAccelBufferPtr bufferAcc;
    UserGyroBufferPtr bufferGyro;
    UserMagnetBufferPtr bufferMag;
    UserGPSBufferPtr bufferGPS;
    int imu_last_launch = 0;
    int gps_last_launch = 0;
    double time = 0.;
    // ==============================
    while (time < end_time) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // ==============================
        // REMOVE IN MAIN DEMO
        // ==============================
        // Get the most recent imu data
        bufferAcc = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        bufferGyro = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
        bufferMag = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();
        if (bufferAcc->Buffer && bufferGyro->Buffer && bufferMag->Buffer &&
            bufferMag->LaunchedCount > imu_last_launch) {
            // Save the imu data to file
            AccelData acc_data = bufferAcc->Buffer[0];
            GyroData gyro_data = bufferGyro->Buffer[0];
            MagnetData mag_data = bufferMag->Buffer[0];

            imu_csv << std::fixed << std::setprecision(6);
            imu_csv << acc_data.X;
            imu_csv << acc_data.Y;
            imu_csv << acc_data.Z;
            imu_csv << gyro_data.Roll;
            imu_csv << gyro_data.Pitch;
            imu_csv << gyro_data.Yaw;
            imu_csv << mag_data.X;
            imu_csv << mag_data.Y;
            imu_csv << mag_data.Z;
            imu_csv << std::endl;
            imu_last_launch = bufferMag->LaunchedCount;
        }

        // Get the most recent gps data
        bufferGPS = gps_normal->GetMostRecentBuffer<UserGPSBufferPtr>();
        if (bufferGPS->Buffer && bufferGPS->LaunchedCount > gps_last_launch) {
            // Save the gps data to file
            GPSData gps_data = bufferGPS->Buffer[0];
            gps_csv << std::fixed << std::setprecision(10);
            gps_csv << gps_data.Latitude;   // Latitude
            gps_csv << gps_data.Longitude;  // Longitude
            gps_csv << gps_data.Altitude;   // Altitude
            gps_csv << gps_data.Time;       // Time
            gps_csv << std::endl;
            gps_last_launch = bufferGPS->LaunchedCount;
        }
        manager->Update();
        // ==============================

        // ADVANCE SYSTEM STATE BY ONE STEP
        sys.DoStepDynamics(2e-3);

        // Irrlicht must finish drawing the frame
        time = sys.GetChTime();
    }

    imu_csv.write_to_file(imu_file);
    gps_csv.write_to_file(gps_file);

    return 0;
}
