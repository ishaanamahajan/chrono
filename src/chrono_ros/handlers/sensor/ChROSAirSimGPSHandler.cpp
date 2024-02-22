// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Huzaifa
// =============================================================================
//
// ROS Handler for communicating gps information from AirSim
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSAirSimGPSHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSAirSimGPSHandler::ChROSAirSimGPSHandler(std::shared_ptr<chrono::sensor::ChGPSSensor> gps,
                                             const std::string& topic_name,
                                             float hdop_init,
                                             float vdop_init,
                                             float hdop_final,
                                             float vdop_final)
    : ChROSAirSimGPSHandler(gps->GetUpdateRate(), gps, topic_name, hdop_init, vdop_init, hdop_final, vdop_final) {}

ChROSAirSimGPSHandler::ChROSAirSimGPSHandler(double update_rate,
                                             std::shared_ptr<chrono::sensor::ChGPSSensor> gps,
                                             const std::string& topic_name,
                                             float hdop_init,
                                             float vdop_init,
                                             float hdop_final,
                                             float vdop_final)
    : ChROSHandler(update_rate),
      m_gps(gps),
      m_topic_name(topic_name),
      m_eph_init(hdop_init),
      m_epv_init(vdop_init),
      m_eph(hdop_init),
      m_epv(vdop_init),
      m_eph_final(hdop_final),
      m_epv_final(vdop_final),
      m_eph_timeConstant(0.9f),
      m_epv_timeConstant(0.9f),
      m_lon_std(0.02f),
      m_lat_std(0.02f),
      m_alt_std(0.02f),
      m_last_time(0.f) {}

bool ChROSAirSimGPSHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterGPSAccess, ChFilterGPSAccessName>(m_gps)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::NavSatFix>(m_topic_name, 1);

    m_gps_msg.header.frame_id = m_gps->GetName();

    return true;
}

void ChROSAirSimGPSHandler::Tick(double time) {
    auto gps_ptr = m_gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (!gps_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "GPS buffer is not ready. Not ticking. \n";
        return;
    }

    GPSData gps_data = gps_ptr->Buffer[0];
    m_gps_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(gps_data.Time);
    m_gps_msg.latitude = gps_data.Latitude;
    m_gps_msg.longitude = gps_data.Longitude;
    m_gps_msg.altitude = gps_data.Altitude;

    // Update the covariance matrix
    // The ChGPSSensor does not currently support covariances, so we'll
    // use the imu message to store a rolling average of the covariance
    m_gps_msg.position_covariance = CalculateCovariance(gps_data);
    m_gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    m_publisher->publish(m_gps_msg);
    m_last_time = time;
}

void ChROSAirSimGPSHandler::FirstOrderFilter(double dt) {
    float alpha = exp(-dt / m_eph_timeConstant);
    // x(k+1) = Ad*x(k) + Bd*u(k)
    m_eph = alpha * m_eph + (1 - alpha) * m_eph_final;
    m_epv = alpha * m_epv + (1 - alpha) * m_epv_final;
}

std::array<double, 9> ChROSAirSimGPSHandler::CalculateCovariance(const GPSData& gps_data) {
    // Get dt
    double dt = gps_data.Time - m_last_time;
    // Update the hdop and vdop
    FirstOrderFilter(dt);
    // Use filter to calculate the covariance -> For now m_epv is not used
    std::array<double, 9> covariance;
    covariance[0] = (m_eph * m_lon_std) * (m_eph * m_lon_std);
    covariance[4] = (m_eph * m_lat_std) * (m_eph * m_lat_std);
    covariance[8] = (2. * m_eph * m_alt_std) * (2. * m_eph * m_alt_std);

    return covariance;
}

}  // namespace ros
}  // namespace chrono