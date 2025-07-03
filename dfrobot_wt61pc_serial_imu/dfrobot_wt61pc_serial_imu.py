#!/usr/bin/env python3
"""
DFRobot WT61PC IMU Sensor Python Library
Corrected version to match C++ behavior exactly

Original C++ library:
Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
License: The MIT License (MIT)
Author: huyujie(yujie.hu@dfrobot.com)
Version: V1.0
Date: 2023-07-10
URL: https://github.com/DFRobot/DFRobot_WT61PC

Python conversion for use with pyserial - Corrected Version
"""

import serial
import time
import struct
from dataclasses import dataclass
from typing import Optional, Tuple


class FrequencyConstants:
    """Data output frequency constants"""
    FREQUENCY_0_1HZ = 0x01
    FREQUENCY_0_5HZ = 0x02
    FREQUENCY_1HZ = 0x03
    FREQUENCY_2HZ = 0x04
    FREQUENCY_5HZ = 0x05
    FREQUENCY_10HZ = 0x06
    FREQUENCY_20HZ = 0x07
    FREQUENCY_50HZ = 0x08
    FREQUENCY_100HZ = 0x09
    FREQUENCY_125HZ = 0x0A
    FREQUENCY_200HZ = 0x0B


@dataclass
class SensorData:
    """Structure to store three-axis sensor data"""
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0
    
    def __str__(self):
        return f"X: {self.X:.3f}, Y: {self.Y:.3f}, Z: {self.Z:.3f}"


class DFRobotWT61PC:
    """DFRobot WT61PC IMU Sensor Python Class - Corrected to match C++ exactly"""
    
    # Packet headers
    HEADER_ACC = 0x51      # Acceleration packet header
    HEADER_GYRO = 0x52     # Angular velocity packet header  
    HEADER_ANGLE = 0x53    # Angle packet header
    HEADER_55 = 0x55
    
    # Timeout in milliseconds (converted to seconds for Python)
    TIMEOUT = 5.0
    
    # Data packet indices
    WT61PC_XL = 2
    WT61PC_XH = 3
    WT61PC_YL = 4
    WT61PC_YH = 5
    WT61PC_ZL = 6
    WT61PC_ZH = 7
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 9600, timeout: float = 1.0):
        """
        Initialize the WT61PC sensor
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Serial communication baud rate (default 9600)
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_timeout = timeout
        
        # Initialize sensor data structures
        self.acc = SensorData()    # Acceleration data (m/s²)
        self.gyro = SensorData()   # Angular velocity data (°/s)
        self.angle = SensorData()  # Angle data (°) - Range: -180° to +180°
        
        # Data packet buffers
        self.received_acc_data = bytearray(11)
        self.received_gyro_data = bytearray(11)
        self.received_angle_data = bytearray(11)
        
        # Frequency modification command template
        self.cmd = bytearray([0xFF, 0xAA, 0x03, FrequencyConstants.FREQUENCY_10HZ, 0x00])
        
        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        
    def connect(self) -> bool:
        """
        Connect to the sensor
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to WT61PC on {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the sensor"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected from WT61PC")
    
    def _read_n(self, length: int) -> int:
        """
        Read n bytes from serial port with timeout - matches C++ readN behavior
        
        Args:
            length: Number of bytes to read
            
        Returns:
            int: Number of bytes actually read
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return 0
            
        start_time = time.time()
        offset = 0
        left = length
        
        while left > 0:
            if time.time() - start_time > self.TIMEOUT:
                break
                
            if self.serial_conn.in_waiting > 0:
                byte_data = self.serial_conn.read(1)
                if byte_data:
                    # Store in temporary buffer for this read operation
                    if not hasattr(self, '_temp_buf'):
                        self._temp_buf = bytearray(11)
                    self._temp_buf[offset] = byte_data[0]
                    offset += 1
                    left -= 1
        
        return offset
    
    def _recv_data(self, buf: bytearray, header: int) -> bool:
        """
        Receive and validate data packet - matches C++ recvData exactly
        
        Args:
            buf: Buffer to store received data
            header: Expected packet header (HEADER_ACC, HEADER_GYRO, or HEADER_ANGLE)
            
        Returns:
            bool: True if valid packet received, False otherwise
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
            
        start_time = time.time()
        ret = False
        
        while not ret:
            if time.time() - start_time > self.TIMEOUT:
                break
                
            # Read one byte looking for HEADER_55
            if self._read_n(1) != 1:
                continue
                
            ch = self._temp_buf[0]
            if ch == self.HEADER_55:
                buf[0] = ch
                
                # Read second header byte
                if self._read_n(1) == 1:
                    ch = self._temp_buf[0]
                    if ch == header:
                        buf[1] = ch
                        
                        # Read remaining 9 bytes directly into buffer
                        bytes_read = 0
                        for i in range(9):
                            if self._read_n(1) == 1:
                                buf[2 + i] = self._temp_buf[0]
                                bytes_read += 1
                            else:
                                break
                        
                        if bytes_read == 9:
                            # Verify checksum
                            if self._get_checksum(buf) == buf[10]:
                                ret = True
                                
        return ret
    
    def _get_checksum(self, buf: bytearray) -> int:
        """
        Calculate checksum for data packet - matches C++ getCS exactly
        
        Args:
            buf: Data packet buffer
            
        Returns:
            int: Calculated checksum
        """
        cs = 0
        for i in range(10):
            cs = cs + buf[i]
        return cs & 0xFF  # Ensure 8-bit result
    
    def available(self) -> bool:
        """
        Check if sensor data is available and read it - matches C++ available exactly
        
        Returns:
            bool: True if data successfully read, False otherwise
        """
        if (self._recv_data(self.received_acc_data, self.HEADER_ACC) and 
            self._recv_data(self.received_gyro_data, self.HEADER_GYRO) and 
            self._recv_data(self.received_angle_data, self.HEADER_ANGLE)):
            
            # Process the data
            self._get_acc(self.received_acc_data)
            self._get_gyro(self.received_gyro_data)
            self._get_angle(self.received_angle_data)
            
            return True
        
        return False
    
    def _get_acc(self, buf: bytearray):
        """
        Calculate acceleration values from raw data - matches C++ getAcc exactly
        
        Args:
            buf: Raw acceleration data packet
        """
        # Use big-endian unsigned 16-bit values, then convert to signed
        x_raw = (buf[self.WT61PC_XH] << 8) | buf[self.WT61PC_XL]
        y_raw = (buf[self.WT61PC_YH] << 8) | buf[self.WT61PC_YL]
        z_raw = (buf[self.WT61PC_ZH] << 8) | buf[self.WT61PC_ZL]
        
        # Convert to signed 16-bit values
        if x_raw > 32767:
            x_raw -= 65536
        if y_raw > 32767:
            y_raw -= 65536
        if z_raw > 32767:
            z_raw -= 65536
        
        # Scale to m/s² (exactly matching C++ calculation)
        self.acc.X = x_raw / 32768.0 * 16.0 * 9.8
        self.acc.Y = y_raw / 32768.0 * 16.0 * 9.8
        self.acc.Z = z_raw / 32768.0 * 16.0 * 9.8
    
    def _get_gyro(self, buf: bytearray):
        """
        Calculate gyroscope values from raw data - matches C++ getGyro exactly
        
        Args:
            buf: Raw gyroscope data packet
        """
        # Use big-endian unsigned 16-bit values, then convert to signed
        x_raw = (buf[self.WT61PC_XH] << 8) | buf[self.WT61PC_XL]
        y_raw = (buf[self.WT61PC_YH] << 8) | buf[self.WT61PC_YL]
        z_raw = (buf[self.WT61PC_ZH] << 8) | buf[self.WT61PC_ZL]
        
        # Convert to signed 16-bit values
        if x_raw > 32767:
            x_raw -= 65536
        if y_raw > 32767:
            y_raw -= 65536
        if z_raw > 32767:
            z_raw -= 65536
        
        # Scale to °/s (exactly matching C++ calculation)
        self.gyro.X = x_raw / 32768.0 * 2000.0
        self.gyro.Y = y_raw / 32768.0 * 2000.0
        self.gyro.Z = z_raw / 32768.0 * 2000.0
    
    def _get_angle(self, buf: bytearray):
        """
        Calculate angle values from raw data - matches C++ getAngle exactly
        
        Args:
            buf: Raw angle data packet
        """
        # Use big-endian unsigned 16-bit values, then convert to signed
        x_raw = (buf[self.WT61PC_XH] << 8) | buf[self.WT61PC_XL]
        y_raw = (buf[self.WT61PC_YH] << 8) | buf[self.WT61PC_YL]
        z_raw = (buf[self.WT61PC_ZH] << 8) | buf[self.WT61PC_ZL]
        
        # Convert to signed 16-bit values
        if x_raw > 32767:
            x_raw -= 65536
        if y_raw > 32767:
            y_raw -= 65536
        if z_raw > 32767:
            z_raw -= 65536
        
        # Scale to degrees: -180° to +180° (exactly matching C++ calculation)
        self.angle.X = x_raw / 32768.0 * 180.0
        self.angle.Y = y_raw / 32768.0 * 180.0
        self.angle.Z = z_raw / 32768.0 * 180.0
    
    def modify_frequency(self, frequency: int):
        """
        Modify the sensor data output frequency - matches C++ modifyFrequency exactly
        
        Args:
            frequency: Frequency constant from FrequencyConstants class
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("Error: Not connected to sensor")
            return
            
        self.cmd[3] = frequency
        self.serial_conn.write(self.cmd)
        print(f"Frequency modified to: 0x{frequency:02X}")
    
    def get_all_data(self) -> Tuple[SensorData, SensorData, SensorData]:
        """
        Get all sensor data (acceleration, gyroscope, angle)
        
        Returns:
            Tuple[SensorData, SensorData, SensorData]: (acc, gyro, angle) data
        """
        return self.acc, self.gyro, self.angle
    
    def get_yaw_0_360(self) -> float:
        """
        Convert yaw angle from -180/+180 to 0-360 degrees
        
        Returns:
            float: Yaw angle in 0-360 degree range
        """
        yaw = self.angle.Z
        if yaw < 0:
            yaw += 360.0
        return yaw


def main():
    """Example usage of the corrected DFRobot WT61PC sensor"""
    
    # Create sensor instance
    sensor = DFRobotWT61PC(port='/dev/ttyUSB0', baudrate=9600)
    
    # Connect to sensor
    if not sensor.connect():
        print("Failed to connect to sensor")
        return
    
    try:
        print("Reading WT61PC sensor data...")
        print("Angle range: -180° to +180° (matches C++ exactly)")
        print("Press Ctrl+C to stop")
        print("-" * 70)
        
        # Optional: Set data output frequency to 10Hz
        sensor.modify_frequency(FrequencyConstants.FREQUENCY_10HZ)
        time.sleep(0.1)  # Wait a bit after frequency change
        
        while True:
            if sensor.available():
                acc, gyro, angle = sensor.get_all_data()
                yaw_360 = sensor.get_yaw_0_360()
                
                print(f"Acceleration (m/s²): {acc}")
                print(f"Gyroscope (°/s):     {gyro}")
                print(f"Angle (°):           {angle}")
                print(f"Yaw 0-360°:          {yaw_360:.3f}")
                print("-" * 70)
                
                time.sleep(0.1)  # Small delay between readings
            else:
                print("No data available, retrying...")
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\nStopping data acquisition...")
    
    finally:
        sensor.disconnect()


if __name__ == "__main__":
    main()