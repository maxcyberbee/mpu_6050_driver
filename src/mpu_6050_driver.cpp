#include "mpu_6050_driver.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

ImuPublisher::ImuPublisher() : Node("imu_node"), file_i2c(0) {
  // Initialize parameters
  this->declare_parameter<int>("smbus", 8);
  this->declare_parameter<int>("polling_rate", 5);
  this->declare_parameter<int>("gyro_dlpf", 4);
  this->declare_parameter<int>("accel_dlpf", 4);
  this->declare_parameter<int>("gyro_fsr", 250);
  this->declare_parameter<int>("accel_fsr", 2);

  // Retrieve parameters
  smbus = this->get_parameter("smbus").as_int();
  auto polling_rate = this->get_parameter("polling_rate").as_int();
  auto gyro_dlpf = this->get_parameter("gyro_dlpf").as_int();
  auto accel_dlpf = this->get_parameter("accel_dlpf").as_int();
  auto gyro_fsr = this->get_parameter("gyro_fsr").as_int();
  auto accel_fsr = this->get_parameter("accel_fsr").as_int();

  // Convert parameters to enums
  auto gyro_dlpf_freq = static_cast<GyroDlpfFrequencyType>(gyro_dlpf);
  auto accel_dlpf_freq = static_cast<AccelDlpfFrequencyType>(accel_dlpf);
  auto gyro_fsr_enum = static_cast<GyroFsrType>(gyro_fsr);
  auto accel_fsr_enum = static_cast<AccelFsrType>(accel_fsr);

  imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu9250", 10);
  timer = this->create_wall_timer(std::chrono::milliseconds(polling_rate),
                                  std::bind(&ImuPublisher::publish_imu, this));
  RCLCPP_INFO(this->get_logger(), "Starting device initialization");
  // Initialize and configure the device
  initialize();
  set_dlpf_frequencies(gyro_dlpf_freq, accel_dlpf_freq);
  set_gyro_fsr(gyro_fsr_enum);
  set_accel_fsr(accel_fsr_enum);

  RCLCPP_INFO(this->get_logger(), "Device initialization complete");
}

int16_t ImuPublisher::read_word_2c(int addr) {
  int16_t val = (buffer[0] << 8) + buffer[1];
  if (val >= 0x8000) {
    return -((65535 - val) + 1);
  } else {
    return val;
  }
}

void ImuPublisher::initialize() {
  RCLCPP_INFO(this->get_logger(), "Opening I2C device file");

  // Open the I2C device file
  file_i2c = open("/dev/i2c-1", O_RDWR);
  if (file_i2c < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device file");
    return;
  }

  // Specify the address of the MPU-6050 we wish to communicate with
  if (ioctl(file_i2c, I2C_SLAVE, ADDR) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to communicate with the MPU-6050");
    return;
  }

  // Initialize the MPU-6050 by writing to the power management register
  char config[2] = {0};
  config[0] = PWR_MGMT_1;
  config[1] = 0;
  write(file_i2c, config, 2);

  RCLCPP_INFO(this->get_logger(), "MPU-6050 successfully initialized");
}

void ImuPublisher::set_dlpf_frequencies(GyroDlpfFrequencyType gyro_freq,
                                        AccelDlpfFrequencyType accel_freq) {
  // Write to the CONFIG register to set the gyro DLPF frequency
  char config[2] = {0};
  config[0] = CONFIG;
  config[1] = static_cast<unsigned char>(gyro_freq);
  write(file_i2c, config, 2);

  // Write to the ACCEL_CONFIG_2 register to set the accel DLPF frequency
  config[0] = ACCEL_CONFIG_2;
  config[1] = static_cast<unsigned char>(accel_freq);
  write(file_i2c, config, 2);
}

void ImuPublisher::set_gyro_fsr(GyroFsrType fsr) {
  // Write to the GYRO_CONFIG register to set the gyro FSR
  char config[2] = {0};
  config[0] = GYRO_CONFIG;
  config[1] = static_cast<unsigned char>(fsr) << 3;
  write(file_i2c, config, 2);
}

void ImuPublisher::set_accel_fsr(AccelFsrType fsr) {
  // Write to the ACCEL_CONFIG register to set the accel FSR
  char config[2] = {0};
  config[0] = ACCEL_CONFIG;
  config[1] = static_cast<unsigned char>(fsr) << 3;
  write(file_i2c, config, 2);
}

void ImuPublisher::deinitialize() {
  RCLCPP_INFO(this->get_logger(), "Deinitializing ImuPublisher");

  // Close the I2C device file
  if (file_i2c > 0) {
    close(file_i2c);
  }
}

void ImuPublisher::publish_imu() {
  auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();

  // Read accelerometer values
  read(file_i2c, buffer, 6);
  float accel_x = 2 * (read_word_2c(ACCEL_XOUT_H) / 32768.0) * 9.81;
  float accel_y = 2 * (read_word_2c(ACCEL_YOUT_H) / 32768.0) * 9.81;
  float accel_z = 2 * (read_word_2c(ACCEL_ZOUT_H) / 32768.0) * 9.81;

  // Read gyro values
  read(file_i2c, buffer, 6);
  float gyro_x = 250 * ((read_word_2c(GYRO_XOUT_H) * M_PI) / 180.0) / 32768.0;
  float gyro_y = 250 * ((read_word_2c(GYRO_YOUT_H) * M_PI) / 180.0) / 32768.0;
  float gyro_z = 250 * ((read_word_2c(GYRO_ZOUT_H) * M_PI) / 180.0) / 32768.0;

  imu_msg->linear_acceleration.x = accel_x;
  imu_msg->linear_acceleration.y = accel_y;
  imu_msg->linear_acceleration.z = accel_z;

  imu_msg->angular_velocity.x = gyro_x;
  imu_msg->angular_velocity.y = gyro_y;
  imu_msg->angular_velocity.z = gyro_z;

  imu_msg->header.stamp = this->now();

  imu_pub->publish(imu_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuPublisher>();
  rclcpp::spin(node);
  node->deinitialize();
  rclcpp::shutdown();

  return 0;
}