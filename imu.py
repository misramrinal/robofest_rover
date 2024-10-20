#!/usr/bin/env python3

import smbus					#import SMself.bus module of I2C
from time import sleep          #import
import math
import rospy
from sensor_msgs.msg import Imu


class main:
    def __init__(self):
        #some MPU6050 Registers and their Address
        PWR_MGMT_1   = 0x6B
        SMPLRT_DIV   = 0x19
        CONFIG       = 0x1A
        GYRO_CONFIG  = 0x1B
        INT_ENABLE   = 0x38
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H  = 0x43
        GYRO_YOUT_H  = 0x45
        GYRO_ZOUT_H  = 0x47

        # Address of the magnetometer sensor
        MAG_XOUT_H = 0x03   # Magnetometer X-axis data high byte register
        MAG_YOUT_H = 0x05   # Magnetometer Y-axis data high byte register
        MAG_ZOUT_H = 0x07   # Magnetometer Z-axis data high byte register
        rospy.init_node('imu_node')
        IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')
        self.bus = smbus.SMBus(1)
        imu_pub = rospy.Publisher('imu/data_raw', Imu,queue_size=10)
        self.Device_Address = 0x68
        self.MAG_ADDRESS = 0x1E

        self.bus.write_byte_data(self.Device_Address, SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.Device_Address, PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.Device_Address, CONFIG, 0)
        self.bus.write_byte_data(self.Device_Address, GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.Device_Address, INT_ENABLE, 1)
        self.bus.write_byte_data(self.MAG_ADDRESS, 0x02, 0x00)
        
        while not rospy.is_shutdown():
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            mag_x = self.read_raw_magnetometer_data(MAG_XOUT_H)
            mag_y = self.read_raw_magnetometer_data(MAG_YOUT_H)
            mag_z = self.read_raw_magnetometer_data(MAG_ZOUT_H)
            
            #Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = acc_x/16384.0
            Ay = acc_y/16384.0
            Az = acc_z/16384.0
            
            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0
            
            Mx = mag_x  # Apply appropriate scaling and calibration for magnetometer data
            My = mag_y  # Apply appropriate scaling and calibration for magnetometer data
            Mz = mag_z 
            
            # roll, pitch, yaw = calculate_roll_pitch_yaw(Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz)
            # print ("Roll=%.2f" %roll, u'\u00b0'+ "/s", "\tPitch=%.2f" %pitch, u'\u00b0'+ "/s", "\tYaw=%.2f" %yaw)
            
            imu_msg.linear_acceleration.x = Ax*9.8
            imu_msg.linear_acceleration.y = Ay*9.8
            imu_msg.linear_acceleration.z = Az*9.8
            
            imu_msg.angular_velocity.x = Gx*0.0174
            imu_msg.angular_velocity.y = Gy*0.0174
            imu_msg.angular_velocity.z = Gz*0.0174
            
            imu_msg.header.stamp = rospy.Time.now()
            imu_pub.publish(imu_msg)
    
            # sleep(1)
        
    def read_raw_data(self,addr):
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)
        value = ((high << 8) | low)
        if(value > 32768):
            value = value - 65536
        return value
    
    def read_raw_magnetometer_data(self,addr):
        high = self.bus.read_byte_data(self.MAG_ADDRESS, addr)
        low = self.bus.read_byte_data(self.MAG_ADDRESS, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value
		
if __name__ == '__main__':
    imu_msg = Imu()
    
    main()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
