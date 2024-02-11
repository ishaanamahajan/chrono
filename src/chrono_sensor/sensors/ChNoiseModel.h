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
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHNOISEMODEL_H
#define CHNOISEMODEL_H
#include "chrono_sensor/ChApiSensor.h"
#include "chrono/core/ChVector.h"
#include "chrono_sensor/utils/ChGPSUtils.h"
#include <random>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Noise model base class
class CH_SENSOR_API ChNoiseModel {
  public:
    /// Class constructor
    ChNoiseModel() {}
    /// Class destructor
    ~ChNoiseModel() {}

    /// Function for adding noise to data
    /// @param data data to augment
    virtual void AddNoise(ChVector<double>& data) = 0;
    virtual void AddNoise(ChVector<double>& data, float last_ch_time, float ch_time) = 0;
    // virtual void AddNoise(chrono::ChVector<float>& gyro, chrono::ChVector<float>& acc) = 0;
};  // class ChNoiseModel

/// Noise model: no noise
class CH_SENSOR_API ChNoiseNone : public ChNoiseModel {
  public:
    /// Class constructor
    ChNoiseNone() {}
    /// Class destructor
    ~ChNoiseNone() {}

    /// Function for adding noise to a set of values. This is empty and adds no noise.
    /// @param data data to augment
    virtual void AddNoise(ChVector<double>& data) {}
    virtual void AddNoise(ChVector<double>& data, float last_ch_time, float ch_time) {}
};

class CH_SENSOR_API ChNoiseNormal : public ChNoiseModel {
  public:
    /// Class constructor
    /// @param mean The mean of the normal distribution
    /// @param stdev The standard deviation of the normal distribution
    ChNoiseNormal(ChVector<double> mean, ChVector<double> stdev);
    // ChNoiseNormal();
    /// Class destructor
    ~ChNoiseNormal() {}

    /// Noise addition function. Adds no noise for this model.
    /// @param data data to augment
    virtual void AddNoise(ChVector<double>& data);
    virtual void AddNoise(ChVector<double>& data, float last_ch_time, float ch_time);

  private:
    std::minstd_rand m_generator;  ///< random number generator
    ChVector<double> m_mean;       ///< mean of the normal distribution
    ChVector<double> m_stdev;      ///< standard deviation of the normal distribution
};

/// IMU Noise model: gaussian drifting noise with noncorrelated equal distributions
class CH_SENSOR_API ChNoiseNormalDrift : public ChNoiseModel {
  public:
    /// Class constructor
    ChNoiseNormalDrift(double updateRate,
                       ChVector<double> mean,
                       ChVector<double> stdev,
                       double drift_bias,
                       double tau_drift);
    // ChNoiseNormalDrift();
    /// Class destructor
    ~ChNoiseNormalDrift() {}

    /// Function for adding noise to data
    /// @param data data to augment
    virtual void AddNoise(ChVector<double>& data);
    virtual void AddNoise(ChVector<double>& data, float last_ch_time, float ch_time);

  private:
    std::minstd_rand m_generator;  ///< random number generator
    ChVector<double> m_bias_prev;

    double m_updateRate;       ///< holding the sensor update rate for use in drift
    ChVector<double> m_mean;   ///< mean of normal distribution for gyroscope
    ChVector<double> m_stdev;  ///< standard deviation of normal distribution for gyroscope
    double m_drift_bias;       ///< bias component of gyroscope drift
    double m_tau_drift;        ///< time constant for gyroscope drift
};

// GPS Noise model based on Random Walks
class CH_SENSOR_API ChNoiseRandomWalks : public ChNoiseModel {
  public:
    /// Class constructor with default tuning parameters
    ChNoiseRandomWalks(float mean, float sigma, float noise_model_update_rate, ChVector<double> gps_reference);
    /// Class constructor with custom tuning parameters
    ChNoiseRandomWalks(float mean,
                       float sigma,
                       float noise_model_update_rate,
                       double max_velocity,
                       double max_acceleration,
                       double max_nudge_proportion,
                       ChVector<double> gps_reference);
    /// Class destructor
    ~ChNoiseRandomWalks() {}

    /// Function for adding noise to data
    /// @param data data to augment
    virtual void AddNoise(ChVector<double>& data) {}
    virtual void AddNoise(ChVector<double>& data, float last_ch_time, float ch_time);

  private:
    std::minstd_rand m_generator;  ///< random number generator
    float m_mean;
    float m_sigma;
    double m_step_size;
    double m_max_velocity;
    double m_max_acceleration;
    double m_max_nudge_proportion;
    ChVector<double> m_gps_reference;
    float m_last_updated_ch_time;
    double m_bx;
    double m_cx;
    double m_by;
    double m_cy;

    double interpolate_weighted_avg(ChVector<double>& data,
                                    float last_ch_time,
                                    float ch_time,
                                    double& meas,
                                    double& dmeas);
    double nudge(double meas, double maxR, double maxD);
    void RandomWalk(double& meas, double& dmeas, double rand);
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif