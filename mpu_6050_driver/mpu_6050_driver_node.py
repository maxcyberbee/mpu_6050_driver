#!/usr/bin/env python

import time
import smbus
import struct
import rclpy
import numpy as np
import math
from sensor_msgs.msg import Imu
from mpu_6050_driver.registers import (
    PWR_MGMT_1,
    ACCEL_XOUT_H,
    ACCEL_YOUT_H,
    ACCEL_ZOUT_H,
    TEMP_H,
    GYRO_XOUT_H,
    GYRO_YOUT_H,
    GYRO_ZOUT_H,
    ACCEL_CONFIG_2,
    CONFIG,
    SAMPLE_RATE_DIVIDER
)


# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr + 1)
    val = (high << 8) + low
    return val


def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


def publish_imu():
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = 2*(read_word_2c(ACCEL_XOUT_H) / 32768.0) * 9.81
    accel_y = 2*(read_word_2c(ACCEL_YOUT_H) / 32768.0) * 9.81
    accel_z = 2*(read_word_2c(ACCEL_ZOUT_H) / 32768.0) * 9.81

    # Calculate a quaternion representing the orientation
    # accel = accel_x, accel_y, accel_z
    # ref = np.array([0, 0, 1])
    # acceln = accel / np.linalg.norm(accel)

    # Read the gyro vals
    gyro_x = 250*((read_word_2c(GYRO_XOUT_H) * math.pi) / 180.0) / 32768.0
    gyro_y = 250*((read_word_2c(GYRO_YOUT_H) * math.pi) / 180.0) / 32768.0
    gyro_z = 250*((read_word_2c(GYRO_ZOUT_H) * math.pi) / 180.0) / 32768.0

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = node.get_clock().now().to_msg()

    imu_pub.publish(imu_msg)


def main():

    global node, imu_pub, IMU_FRAME, bus, ADDR
    rclpy.init()

    node = rclpy.create_node("imu_node")
    timer_period = 0.005
    

    bus = smbus.SMBus(8)
    ADDR = 0x68
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = "imu_link"

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
    bus.write_byte_data(ADDR, CONFIG, 4) # gyro LPF
    bus.write_byte_data(ADDR, ACCEL_CONFIG_2, 4) # accel LPF
    bus.write_byte_data(ADDR, SAMPLE_RATE_DIVIDER,5) # rate

    imu_pub = node.create_publisher(Imu, "imu9250", 10)
    
    node.create_timer(timer_period, publish_imu)
    while rclpy.ok():
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
