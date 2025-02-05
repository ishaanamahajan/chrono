// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Harry Zhang   Aaron Young
// =============================================================================
//
// Demo to show the use of Chrono::Vehicle with ROS
//
// =============================================================================

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "chrono/core/ChTypes.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"

#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"

#include <chrono>
#include <random>
using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;
// using namespace chrono::vehicle::artvehicle;
//  =============================================================================
//  Initial vehicle location and orientation
ChVector<> initLoc(-1.0, -1.0, 0.5);
ChVector<> initLoc_flw(0, 0.0, 0.5);
ChQuaternion<> initRot = Q_from_AngZ(0.0f);
// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;     // terrain height (FLAT terrain only)
double terrainLength = 10.0;  // size in X direction
double terrainWidth = 10.0;   // size in Y direction

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;
// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;
// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;
// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");

// sensor params
unsigned int image_width = 1080;
unsigned int image_height = 720;
float fov = (float)CH_C_PI / 2.;
int alias_factor = 1;
float lag = 0.0f;
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Simulation step size
double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // SetChronoDataPath(CHRONO_DATA_DIR);
    artcar::ARTcar vehicle;
    // vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "/vehicle/");
    vehicle.SetContactMethod(contact_method);
    vehicle.SetChassisCollisionType(chassis_collision_type);
    vehicle.SetChassisFixed(false);
    vehicle.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    vehicle.SetTireType(tire_model);
    vehicle.SetTireStepSize(step_size);
    vehicle.SetMaxMotorVoltageRatio(0.09f);
    vehicle.SetStallTorque(0.3f);
    vehicle.SetTireRollingResistance(0.05f);
    vehicle.Initialize();

    VisualizationType tire_vis_type = VisualizationType::MESH;
    std::cout << "Initialization completed 1." << std::endl;

    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);
    vehicle.SetTireVisualizationType(tire_vis_type);

    // Containing system
    auto system = vehicle.GetSystem();

    // Add box in front of the vehicle
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({1.0, 0.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);

    // Create the terrain
    RigidTerrain terrain(system, vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle.GetVehicle());

    // Create a sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(system);
    manager->scene->AddPointLight({100, 100, 100}, {0.4f, 0.4f, 0.4f}, 500);
    // Set the background to an environment map
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);
    std::cout << "Initialization completed 3." << std::endl;

    // Add camera
    auto cam_pose = chrono::ChFrame<double>({-5.304, 0, 1.0}, Q_from_AngAxis(0.1, {0, 1.25, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassis()->GetBody(),  // body camera is attached to
                                                         10,                               // update rate in Hz
                                                         cam_pose,                         // offset pose
                                                         image_width,                      // image width
                                                         image_height,                     // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         false);        // use global illumination or not

    cam->SetName("Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(0.0f);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Camera"));
    std::cout << "Initialization completed 4." << std::endl;
    // cam->PushFilter(chrono_types::make_shared<ChFilterSave>("./cam1/"));

    manager->AddSensor(cam);
    // std::cout << "Initialization completed 5." << std::endl;
    // manager->Update();
    //------------

    // auto cam = chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassis()->GetBody(), 30,
    // chrono::ChFrame<double>({-5.304, 0, 1.0}, Q_from_AngAxis(0.1, {0, 1.25, 0})) , 1280, 720, CH_C_PI / 3.);
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));
    // std::cout << "Initialization completed 5." << std::endl;
    // manager->AddSensor(cam);

    auto sensor_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {0, 0, 0}));
    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    ChVector<> gps_reference(-89.400, 43.070, 260.0);

    ChVector<double> gyro_noise_mean(0, 0, 0);
    ChVector<double> gyro_noise_stdev(0.05, 0.05, 0.05);  // degrees per second
    ChVector<double> accel_noise_mean(0.0, 0.0, 0.0);
    ChVector<double> accel_noise_stdev(0.01, 0.01, 0.01);  // m/s²

    auto gyro_noise_model = chrono_types::make_shared<ChNoiseNormal>(gyro_noise_mean, gyro_noise_stdev);
    auto accel_noise_model = chrono_types::make_shared<ChNoiseNormal>(accel_noise_mean, accel_noise_stdev);

    ChFrame<double> sensor_frame(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

    // Accelerometer
    auto accelerometer = chrono_types::make_shared<ChAccelerometerSensor>(vehicle.GetChassisBody(), 100,
                                                                          sensor_offset_pose, accel_noise_model);
    accelerometer->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    manager->AddSensor(accelerometer);

    // Gyroscope
    auto gyroscope = chrono_types::make_shared<ChGyroscopeSensor>(vehicle.GetChassisBody(), 100, sensor_offset_pose,
                                                                  gyro_noise_model);
    gyroscope->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    manager->AddSensor(gyroscope);

    // Magnetometer
    auto magnetometer = chrono_types::make_shared<ChMagnetometerSensor>(vehicle.GetChassisBody(), 100,
                                                                        sensor_offset_pose, noise_none, gps_reference);
    magnetometer->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    manager->AddSensor(magnetometer);

    ChVector<double> gps_noise_mean(0, 0, 0);
    ChVector<double> gps_noise_stdev(0.017, 0.017, 0.017);  // meters

    auto gps_noise_model = chrono_types::make_shared<ChNoiseNormal>(gps_noise_mean, gps_noise_stdev);

    auto gps = chrono_types::make_shared<ChGPSSensor>(vehicle.GetChassis()->GetBody(), 10.f, sensor_offset_pose,
                                                      gps_reference, noise_none);
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps);
    manager->Update();

    // Add visualization
    // Create the vehicle Irrlicht interface

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("ARTcar Demo");
    // Point on chassis tracked by the camera
    ChVector<> trackPoint(0.0, 0.0, 0.2);
    vis->SetChaseCamera(trackPoint, 1.5, 0.05);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle.GetVehicle());

    double t_end = 300;
    double time = 0;
    double t = 20;
    double interval = 2;

    // Rendering stuff
    double render_step_size = 1.0 / 50;  // FPS = 50
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    while (time < t_end) {
        // Get driver inputs
        DriverInputs driver_inputs = {0, 0.5, 0};
        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();

        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            render_frame++;
        }

        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);

        // update sensor manager
        manager->Update();

        // Increment frame number
        step_number++;
    }

    return 0;
}