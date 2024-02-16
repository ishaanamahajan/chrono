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
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include <chrono>

namespace chrono {
namespace sensor {

ChNoiseNormal::ChNoiseNormal(ChVector<double> mean, ChVector<double> stdev)
    : m_mean(mean), m_stdev(stdev), ChNoiseModel() {
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
}

void ChNoiseNormal::AddNoise(ChVector<double>& data) {
    std::normal_distribution<double> dist_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_z(m_mean.z(), m_stdev.z());
    data += ChVector<double>(dist_x(m_generator), dist_y(m_generator), dist_z(m_generator));
}

void ChNoiseNormal::AddNoise(ChVector<double>& data, float last_ch_time, float ch_time) {
    AddNoise(data);
}

ChNoiseNormalDrift::ChNoiseNormalDrift(double updateRate,
                                       ChVector<double> mean,
                                       ChVector<double> stdev,
                                       double drift_bias,
                                       double tau_drift)
    : m_updateRate(updateRate),
      m_mean(mean),
      m_stdev(stdev),
      m_drift_bias(drift_bias),
      m_tau_drift(tau_drift),
      ChNoiseModel() {
    m_bias_prev = {0, 0, 0};
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
}

void ChNoiseNormalDrift::AddNoise(ChVector<double>& data) {
    std::normal_distribution<double> dist_a_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_a_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_a_z(m_mean.z(), m_stdev.z());
    ChVector<double> eta_a = {dist_a_x(m_generator), dist_a_y(m_generator), dist_a_z(m_generator)};

    ChVector<double> eta_b = {0, 0, 0};
    if (m_tau_drift > std::numeric_limits<double>::epsilon() && m_drift_bias > std::numeric_limits<double>::epsilon()) {
        std::normal_distribution<double> dist_b(0.0, m_drift_bias * sqrt(1 / (m_updateRate * m_tau_drift)));
        eta_b = {dist_b(m_generator), dist_b(m_generator), dist_b(m_generator)};
    }
    m_bias_prev += eta_b;
    data += eta_a + m_bias_prev;
}

void ChNoiseNormalDrift::AddNoise(ChVector<double>& data, float last_ch_time, float ch_time) {
    AddNoise(data);
}

// RandomWalks class constructor with default parameters
ChNoiseRandomWalks::ChNoiseRandomWalks(float mean,
                                       float sigma,
                                       float noise_model_update_rate,
                                       ChVector<double> gps_reference)
    : m_mean(mean),
      m_sigma(sigma),
      m_step_size((float)1 /
                  noise_model_update_rate),  // Note: noise_model_update_rate likely differs from GPS update rate.
      m_max_velocity(0.03),
      m_max_acceleration(0.005),
      m_gps_reference(gps_reference),
      m_last_updated_ch_time(0),
      ChNoiseModel() {
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
}

// RandomWalks class constructor with custom parameters
ChNoiseRandomWalks::ChNoiseRandomWalks(float mean,
                                       float sigma,
                                       float noise_model_update_rate,
                                       double max_velocity,
                                       double max_acceleration,
                                       ChVector<double> gps_reference)
    : m_mean(mean),
      m_sigma(sigma),
      m_step_size((float)1 /
                  noise_model_update_rate),  // Note: noise_model_update_rate likely differs from GPS update rate.
      m_max_velocity(max_velocity),
      m_max_acceleration(max_acceleration),
      m_gps_reference(gps_reference),
      m_last_updated_ch_time(0),
      ChNoiseModel() {
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    m_prev_error_v = ChVector<double>(0, 0, 0);
    m_prev_error_p = ChVector<double>(0, 0, 0);
}

void ChNoiseRandomWalks::AddNoise(ChVector<double>& data, float last_ch_time, float next_ch_time) {
    // GPS2Cartesian(data, m_gps_reference);
    // Initialize integration state

    double curr_time = m_last_updated_ch_time;

    // m_prev_error_p = ChVector<double>(data.x(), data.y(), data.z());
    // std::cout << curr_time << " " << next_ch_time << std::endl;
    // std::cout << "step size " << m_step_size << std::endl;
    // Main integration loop
    while (curr_time < next_ch_time) {
        // Determine timestep for this iteration
        double time_step = std::min(m_step_size, next_ch_time - curr_time);

        double maxD = 1.;
        // Sample either 1 or -1
        std::uniform_int_distribution<int> dist_sample(-1, 1);
        int sample = dist_sample(m_generator);
        // int sample = -1;
        // double c_x = sample * data.x() / (maxD + data.x());
        // double c_y = sample * data.y() / (maxD + data.y());
        // double c_z = sample * data.z() / (maxD + data.z());
        sample = -1;
        double c_x = sample * m_prev_error_p.x() / (maxD);
        double c_y = sample * m_prev_error_p.y() / (maxD);
        double c_z = sample * m_prev_error_p.z() / (maxD);

        // Limit c's to maxR
        // double maxR = 1.;
        // c_x = (c_x > maxR) ? maxR : c_x;
        // c_x = (c_x < -maxR) ? -maxR : c_x;
        // c_y = (c_y > maxR) ? maxR : c_y;
        // c_y = (c_y < -maxR) ? -maxR : c_y;
        // c_z = (c_z > maxR) ? maxR : c_z;
        // c_z = (c_z < -maxR) ? -maxR : c_z;

        // // Apply another weight
        c_x = m_sigma * 0.2 * c_x;
        c_y = m_sigma * 0.2 * c_y;
        c_z = m_sigma * 0.2 * c_z;

        // Generate white noise
        std::normal_distribution<double> dist_x(c_x, m_sigma);
        std::normal_distribution<double> dist_y(c_y, m_sigma);
        std::normal_distribution<double> dist_z(c_z, m_sigma);

        ChVector<double> white_noise = {dist_x(m_generator), dist_y(m_generator), dist_z(m_generator)};

        // Ensure acceleration norm is less than max_acceleration
        // if (white_noise.Length() > m_max_acceleration) {
        //     white_noise.Normalize();
        //     white_noise *= m_max_acceleration;
        // }

        // std::cout << "white noise " << white_noise.x() << " " << white_noise.y() << " " << white_noise.z() <<
        // std::endl;

        // Limit all axis of white noise to m_max_acceleration
        white_noise.x() = (white_noise.x() > m_max_acceleration) ? m_max_acceleration : white_noise.x();
        white_noise.x() = (white_noise.x() < -m_max_acceleration) ? -m_max_acceleration : white_noise.x();
        white_noise.y() = (white_noise.y() > m_max_acceleration) ? m_max_acceleration : white_noise.y();
        white_noise.y() = (white_noise.y() < -m_max_acceleration) ? -m_max_acceleration : white_noise.y();
        white_noise.z() = (white_noise.z() > m_max_acceleration) ? m_max_acceleration : white_noise.z();
        white_noise.z() = (white_noise.z() < -m_max_acceleration) ? -m_max_acceleration : white_noise.z();

        // std::cout << "white noise " << white_noise.x() << " " << white_noise.y() << " " << white_noise.z() <<
        // std::endl;

        // Integrate to get random walk contributions
        m_prev_error_v += white_noise * time_step;
        // If the norm of velocity is greater than max_velocity, set to max velocity
        if (m_prev_error_v.Length() > m_max_velocity) {
            m_prev_error_v = m_prev_error_v / m_prev_error_v.Length() * m_max_velocity;
        }

        // Check all components of velocity and set to 0 if greater than max_velocity
        // m_prev_error_v.x() = (m_prev_error_v.x() > m_max_velocity) ? 0 : m_prev_error_v.x();
        // m_prev_error_v.x() = (m_prev_error_v.x() < -m_max_velocity) ? 0 : m_prev_error_v.x();
        // m_prev_error_v.y() = (m_prev_error_v.y() > m_max_velocity) ? 0 : m_prev_error_v.y();
        // m_prev_error_v.y() = (m_prev_error_v.y() < -m_max_velocity) ? 0 : m_prev_error_v.y();
        // m_prev_error_v.z() = (m_prev_error_v.z() > m_max_velocity) ? 0 : m_prev_error_v.z();
        // m_prev_error_v.z() = (m_prev_error_v.z() < -m_max_velocity) ? 0 : m_prev_error_v.z();

        m_prev_error_p += m_prev_error_v * time_step;
        // std::cout << "prev_error_p " << m_prev_error_p.x() << " " << m_prev_error_p.y() << " " << m_prev_error_p.z()
        //           << std::endl;

        // Update current time
        curr_time += time_step;
    }
    m_last_updated_ch_time = next_ch_time;
    // Apply accumulated position error to data
    data += m_prev_error_p;
}

// void ChNoiseRandomWalks::AddNoise(ChVector<double>& data, float last_ch_time, float ch_time) {
//     std::cout << "data before transform " << data.x() << " " << data.y() << std::endl;
//     // Noise is generated in Meters, convert GPS coordinates to relative x and y position from reference point.
//     // printf("data before transform %lf\n 	%lf\n", data.x(), data.y());
//     GPS2Cartesian(data, m_gps_reference);

//     // m_bx = data.x();
//     // m_by = data.y();
//     double noise_x = interpolate_weighted_avg(data, last_ch_time, ch_time, m_bx, m_cx);

//     double noise_y = interpolate_weighted_avg(data, last_ch_time, ch_time, m_by, m_cy);
//     // printf("	m_bx: %lf\n 	m_cx: %lf\n", *m_bx, *m_cx);
//     data.x() = noise_x;
//     data.y() = noise_y;

//     // Convert data points back to coordinates for GPS output.
//     // Cartesian2GPS(data, m_gps_reference);
//     // printf("data.x(): %lf\n data.y() %lf\n", data.x(), data.y());
// }

// void ChNoiseRandomWalks::RandomWalk(double& meas, double& dmeas, double rand) {
//     double maxv = m_max_velocity;
//     double maxNudge = m_sigma * m_max_nudge_proportion;
//     double maxa = m_max_acceleration;
//     double n = maxNudge * nudge(meas, 1, 1);
//     double a = rand + n;
//     // printf("\n\n The m_sigma is: %f\n\n\"", m_sigma);

//     // Limit a between -maxa and maxa
//     a = (a > maxa) ? maxa : a;
//     a = (a < -maxa) ? -maxa : a;

//     // Make a the maximum value it is allowed to be, while maintaining it's sign.
//     // if (abs(a) > maxa)
//     //     a = maxa * ((a > 0) - (a < 0));
//     // printf("noise added to meas: %f\n", rand);
//     // printf("nudge added to meas: %f\n", n);
//     // printf("		meas before noise: %f\n", *meas);
//     dmeas = dmeas + a;

//     dmeas = (dmeas > maxv) ? maxv : dmeas;
//     dmeas = (dmeas < -maxv) ? -maxv : dmeas;

//     meas = meas + dmeas + a;

//     // printf("		meas after noise: %f\n", *meas);

//     // Limit dmeas between -maxv and maxv

//     // if (abs(*dmeas) > maxv)
//     //     *dmeas = maxv * ((*dmeas > 0) - (*dmeas < 0));
// }

// double ChNoiseRandomWalks::interpolate_weighted_avg(ChVector<double>& data,
//                                                     float last_ch_time,
//                                                     float ch_time,
//                                                     double& meas,
//                                                     double& dmeas) {
//     // const int num_grid_pts = 16; // Had to change to const
//     // int size_one_d = sqrt(num_grid_pts);
//     // double grid_pts_val[num_grid_pts];
//     std::normal_distribution<double> distribution(m_mean, m_sigma);

//     double t = last_ch_time;
//     double h;
//     while (t < ch_time) {
//         h = std::min(m_step_size, ch_time - t);
//         double rand = distribution(m_generator);
//         RandomWalk(meas, dmeas, rand);
//         t += h;
//     }
// int steps = (ch_time - last_ch_time) / m_step_size;
// if (steps > 0) {
//     m_last_updated_ch_time = ch_time;
// } else {
//     // if the last updated time equals chrono time already, model is calculating noise for second element of X Y
//     // pair and should also trigger a noise model update.
//     if (ch_time == m_last_updated_ch_time || (ch_time - m_last_updated_ch_time) / m_step_size > 0) {
//         steps++;
//         m_last_updated_ch_time = ch_time;
//     }
// }
// printf("For update rate: %lf\n", 1 / m_step_size);
// printf("ch_time: %f\n last_ch_time: %f\n m_step_size: %f\n", ch_time, last_ch_time, m_step_size);
// printf("Number of steps in chrono simulation refresh gap %d\n", steps);

// for(int i=0; i<num_grid_pts;i++){
// for (int j = 0; j < steps; j++) {
//     double rand = distribution(m_generator);
//     RandomWalk(meas, dmeas, rand);
//     // RandomWalk(meas+i, dmeas+i, rand);
// }
// grid_pts_val[i] = meas[i];
//}
// double weight = (double) 1 / num_grid_pts;
// double result = 0;
// for(int i = 0; i < num_grid_pts; i++){
//	    result += grid_pts_val[i] * weight;
//}
// printf("%f\n",result);

// return result;
// return meas;
// }

// double ChNoiseRandomWalks::nudge(double meas, double maxR, double maxD) {
//     double myNudge = -meas / maxD;
//     if (abs(myNudge) >= maxR)
//         myNudge = maxR * ((myNudge > 0) - (myNudge < 0));
//     return myNudge;
// }

}  // namespace sensor
}  // namespace chrono