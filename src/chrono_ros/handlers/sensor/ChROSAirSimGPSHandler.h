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
// ROS Handler for communicating gps information based on AirSim
//
// =============================================================================

#ifndef CH_ROS_AirSim_GPS_HANDLER
#define CH_ROS_AirSim_GPS_HANDLER

#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_ros/ChROSHandler.h"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChGPSSensor to ROS. Will publish sensor_msgs::msg::NavSatFix.
class ChROSAirSimGPSHandler : public ChROSGPSHandler {
  public:
    /// Constructor. The update rate is set to gps->GetUpdateRate().
    /// The update rate corresponds to the sensor's update rate.
    ChROSAirSimGPSHandler(std::shared_ptr<chrono::sensor::ChGPSSensor> gps,
                          const std::string& topic_name,
                          float hdop_init,
                          float vdop_init,
                          float hdop_final,
                          float vdop_final);

    /// Full constructor. Takes a ChGPSSensor, update rate, and topic name.
    ChROSAirSimGPSHandler(double update_rate,
                          std::shared_ptr<chrono::sensor::ChGPSSensor> gps,
                          const std::string& topic_name,
                          float hdop_init,
                          float vdop_init,
                          float hdop_final,
                          float vdop_final);

  protected:
    virtual void Tick(double time) override;

  private:
    /// Helper function to calculate the decay in hdop and vdop
    /// Inspired from AirSim
    /// https://github.com/microsoft/AirSim/blob/main/AirLib/include/common/FirstOrderFilter.hpp
    void FirstOrderFilter(double dt);

    /// Helper function to calculate the covariance of the accelerometer
    /// ChGPSSensor currently doesn't support covariance
    /// This is directly inspired from AirSim calculation of hdop's that are
    /// used to calculate the covariance of the gps data using code from this driver

    /// https://github.com/ros-drivers/nmea_navsat_driver/blob/06d71c7c5995fc8ef5579906d1d8097dbacf2c32/src/libnmea_navsat_driver/driver.py#L180C12-L198C60
    std::array<double, 9> CalculateCovariance(const chrono::sensor::GPSData& gps_data);

  private:
    std::shared_ptr<chrono::sensor::ChGPSSensor> m_gps;  ///< handle to the gps sensor

    const std::string m_topic_name;                                         ///< name of the topic to publish to
    sensor_msgs::msg::NavSatFix m_gps_msg;                                  ///< message to publish
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_publisher;  ///< the publisher for the gps message

    // std::array<double, 3> m_running_average;

    /// Parameters required for eph and epv calculation
    float m_eph_init;   // Initial horizontal dilution of precision
    float m_epv_init;   // Initial vertical dilution of precision
    float m_eph_final;  // Final horizontal dilution of precision
    float m_epv_final;  // Final vertical dilution of precision

    float m_eph;  // Horizontal dilution of precision
    float m_epv;  // Vertical dilution of precision

    float
        m_eph_timeConstant;  // Time constant for horizontal dilution of precision -> Used to improve eph exponentially
    float m_epv_timeConstant;  // Time constant for vertical dilution of precision -> Used to improve epv exponentially

    float m_lon_std;  // Constant based on type of gps sensor (see nmea_navsat_driver)
    float m_lat_std;  // Constant based on type of gps sensor (see nmea_navsat_driver)
    float m_alt_std;  // Constant based on type of gps sensor (see nmea_navsat_driver)

    double m_last_time;  // Store last time to calculate dt needed in hdop/vdop calculation
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
