#ifndef MPU_6050_DRIVER_H
#define MPU_6050_DRIVER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <math.h>
#include <memory>

// MPU-6050 Registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_H 0x41
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define ACCEL_CONFIG_2 0x1C
#define CONFIG 0x1B
#define SAMPLE_RATE_DIVIDER 0x19

// MPU-6050 I2C Address
#define ADDR 0x68
#ifndef MPU_6050_DRIVER_H
#define MPU_6050_DRIVER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuPublisher : public rclcpp::Node {
public:
  ImuPublisher();

  enum class GyroDlpfFrequencyType : unsigned char {
    F_5HZ = 0,
    F_10Hz = 1,
    F_20Hz = 2,
    F_41HZ = 3,
    F_92HZ = 4,
    F_184HZ = 5,
    F_250HZ = 6
  };

  enum class AccelDlpfFrequencyType : unsigned char {
    F_5HZ = 0,
    F_10Hz = 1,
    F_21HZ = 2,
    F_44HZ = 3,
    F_99HZ = 4,
    F_218HZ = 5
  };

  enum class GyroFsrType : unsigned char {
    DPS_250 = 0,
    DPS_500 = 1,
    DPS_1000 = 2,
    DPS_2000 = 3
  };

  enum class AccelFsrType : unsigned char {
    G_2 = 0,
    G_4 = 1,
    G_8 = 2,
    G_16 = 3
  };

private:
  void publish_imu();
  void initialize();
  void deinitialize();
  void set_dlpf_frequencies(GyroDlpfFrequencyType gyro_freq,
                            AccelDlpfFrequencyType accel_freq);
  void set_gyro_fsr(GyroFsrType fsr);
  void set_accel_fsr(AccelFsrType fsr);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::TimerBase::SharedPtr timer;
  int file_i2c;
  int smbus;
};

#endif
