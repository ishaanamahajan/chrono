// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Ryan Hanson
// =============================================================================
//
// Chrono demonstration of a GPS using the Random Walks Noise model at 3
// different update rates
//
// Attaches GPS to a stationary point, making comparing simulated expirements
// to a stationary gps operating in the real world comparable.
//
// OUTPUT CSV Format: Longitude Lattitude Altitude Time
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <memory>

// include "chrono/assets/ChTriangleMeshShape.h"
// include "chrono/assets/ChVisualMaterial.h"
// include "chrono/assets/ChVisualShape.h"
// include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
// include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
// using namespace chrono::geometry;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// GPS parameters
// -----------------------------------------------------------------------------
// Noise model attached to the sensor
enum GPSNoiseModel {
    NORMAL,    // individually parameterized independent gaussian distribution
    GPS_NONE,  // no noise model
    GPS_RANDOMWALK
};
// PSNoiseModel gps_noise_type = GPS_NONE;
GPSNoiseModel gps_noise_type = GPS_RANDOMWALK;
// GPS update rate in Hz
int gps_update_rate1 = 1;
int gps_update_rate2 = 10;
int gps_update_rate3 = 100;

// GPS noise model update rate in Hz (for accuracy of noise generation, this may differ from GPS update rate).
int gps_noise_model_update_rate1 = 100;
int gps_noise_model_update_rate2 = 100;
int gps_noise_model_update_rate3 = 100;

// Camera's horizontal field of view
float fov = 1.408f;

// GPS lag (in seconds) between sensing and when data becomes accessible
float gps_lag = 0;

// Collection time (in seconds) of eacn sample
float gps_collection_time = 0;

// Origin used as the gps reference point (Longitude, Lattitude, Altittude)
// Located in Madison, WI
ChVector<> gps_reference1(-89.412240758, 43.071986683, 0);
ChVector<> gps_reference2(-89.412240758, 43.071986683, 0);
ChVector<> gps_reference3(-89.412240758, 43.071986683, 0);
// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 100.0f;

// Save data
bool save = true;

// Output directories
const std::string out_dir = "SENSOR_OUTPUT/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;
    sys.Set_G_acc({0, 0, -9.81});

    // -------------------------------
    // Create a stationary system to attach GPS Sensor to.
    // -------------------------------
    auto base = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1, true, false);
    base->SetPos(ChVector<>(0, 0, 0));
    base->SetBodyFixed(true);  // the base does not move!
    sys.Add(base);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    // ---------------------------------------------
    // Create a GPS and add it to the sensor manager
    // ---------------------------------------------
    auto gps_noise_model1 = chrono_types::make_shared<ChNoiseRandomWalks>(0, 0.16, gps_noise_model_update_rate1, 0.03,
                                                                          0.05, gps_reference1);

    auto gps_noise_model2 = chrono_types::make_shared<ChNoiseRandomWalks>(0, 0.16, gps_noise_model_update_rate2, 0.03,
                                                                          0.05, gps_reference2);
    auto gps_noise_model3 = chrono_types::make_shared<ChNoiseRandomWalks>(0, 0.16, gps_noise_model_update_rate3, 0.03,
                                                                          0.05, gps_reference3);
    // GPS parameters are the same for all 3 GPS sensors, except the Noise model update rate and GPS update rate.
    auto gps_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    // Add gps1 sensor
    auto gps1 = chrono_types::make_shared<ChGPSSensor>(
        base,              // body to which the GPS is attached
        gps_update_rate1,  // update rate
        gps_offset_pose,   // offset pose from body
        gps_reference1,    // reference GPS location (GPS coordinates of simulation origin)
        gps_noise_model1   // noise model to use for adding GPS noise
    );
    gps1->SetName("GPS1");
    gps1->SetLag(gps_lag);
    gps1->SetCollectionWindow(gps_collection_time);
    gps1->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps1);                                          // Add GPS sensor to the sensor manager

    // Add gps2 sensor
    auto gps2 = chrono_types::make_shared<ChGPSSensor>(
        base,              // body to which the GPS is attached
        gps_update_rate2,  // update rate
        gps_offset_pose,   // offset pose from body
        gps_reference2,    // reference GPS location (GPS coordinates of simulation origin)
        gps_noise_model2   // noise model to use for adding GPS noise
    );
    gps2->SetName("GPS2");
    gps2->SetLag(gps_lag);
    gps2->SetCollectionWindow(gps_collection_time);
    gps2->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps2);                                          // Add GPS sensor to the sensor manager

    // Add gps3 sensor
    auto gps3 = chrono_types::make_shared<ChGPSSensor>(
        base,              // body to which the GPS is attached
        gps_update_rate3,  // update rate
        gps_offset_pose,   // offset pose from body
        gps_reference3,    // reference GPS location (GPS coordinates of simulation origin)
        gps_noise_model3   // noise model to use for adding GPS noise
    );
    gps3->SetName("GPS3");
    gps3->SetLag(gps_lag);
    gps3->SetCollectionWindow(gps_collection_time);
    gps3->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps3);                                          // Add GPS sensor to the sensor manager

    // -----------------
    // Initialize output
    // -----------------

    std::string gps_file = out_dir + "gps/";

    if (!filesystem::create_directory(filesystem::path(gps_file))) {
        std::cout << "Error creating directory " << gps_file << std::endl;
        return 1;
    }

    // Create a CSV writer to record the GPS data
    std::string gps1_file = gps_file;
    std::string gps2_file = gps_file;
    std::string gps3_file = gps_file;
    gps1_file += "1HZ_stationary_gps.csv";
    gps2_file += "10HZ_stationary_gps.csv";
    gps3_file += "100HZ_stationary_gps.csv";
    utils::CSV_writer gps1_csv(" ");
    utils::CSV_writer gps2_csv(" ");
    utils::CSV_writer gps3_csv(" ");

    // ---------------
    // Simulate system
    // ---------------
    float ch_time = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    UserGPSBufferPtr bufferGPS1;
    UserGPSBufferPtr bufferGPS2;
    UserGPSBufferPtr bufferGPS3;

    int gps1_last_launch = 0;
    int gps2_last_launch = 0;
    int gps3_last_launch = 0;

    /*
     *Expirement: 3 GPSs run for the same time duration and at the same location, but with differing GPS and noise model
     *update rates.
     */
    while (ch_time < end_time) {
        // Get the most recent gps data for gps1
        bufferGPS1 = gps1->GetMostRecentBuffer<UserGPSBufferPtr>();
        if (bufferGPS1->Buffer && bufferGPS1->LaunchedCount > gps1_last_launch) {
            // Save the gps data to file
            GPSData gps1_data = bufferGPS1->Buffer[0];
            gps1_csv << std::fixed << std::setprecision(10);
            gps1_csv << gps1_data.Longitude;  // Longitude
            gps1_csv << gps1_data.Latitude;   // Latitude
            gps1_csv << gps1_data.Altitude;   // Altitude
            gps1_csv << gps1_data.Time;       // Time
            gps1_csv << std::endl;
            gps1_last_launch = bufferGPS1->LaunchedCount;
        }

        // Get the most recent gps data for gps2
        bufferGPS2 = gps2->GetMostRecentBuffer<UserGPSBufferPtr>();
        if (bufferGPS2->Buffer && bufferGPS2->LaunchedCount > gps2_last_launch) {
            // Save the gps data to file
            GPSData gps2_data = bufferGPS2->Buffer[0];
            gps2_csv << std::fixed << std::setprecision(10);
            gps2_csv << gps2_data.Longitude;  // Longitude
            gps2_csv << gps2_data.Latitude;   // Latitude
            gps2_csv << gps2_data.Altitude;   // Altitude
            gps2_csv << gps2_data.Time;       // Time
            gps2_csv << std::endl;
            gps2_last_launch = bufferGPS2->LaunchedCount;
        }

        // Get the most recent gps data for gps3
        bufferGPS3 = gps3->GetMostRecentBuffer<UserGPSBufferPtr>();
        if (bufferGPS3->Buffer && bufferGPS3->LaunchedCount > gps3_last_launch) {
            // Save the gps data to file
            GPSData gps3_data = bufferGPS3->Buffer[0];
            gps3_csv << std::fixed << std::setprecision(10);
            gps3_csv << gps3_data.Longitude;  // Longitude
            // printf("\nGPS 3: write long to file as %lf\n\n", gps3_data.Longitude);
            gps3_csv << gps3_data.Latitude;  // Latitude
            // printf("\nGPS 3: write lat to file as %lf\n\n", gps3_data.Latitude);
            gps3_csv << gps3_data.Altitude;  // Altitude
            gps3_csv << gps3_data.Time;      // Time
            gps3_csv << std::endl;
            gps3_last_launch = bufferGPS3->LaunchedCount;
        }

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
        // std::cout << "Simulated Time: " << ch_time << std::endl;
    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    gps1_csv.write_to_file(gps1_file);
    gps2_csv.write_to_file(gps2_file);
    gps3_csv.write_to_file(gps3_file);

    return 0;
}