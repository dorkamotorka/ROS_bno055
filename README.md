# ros_bno055
This is a C++ ROS Driver for operating a BOSCH BNO055 IMU sensor via I2C on a Raspberry Pi.

It was

## Dependencies

```
  git clone --single-branch --branch master https://github.com/tp4348/ros_bno055.git

  cd .. 

  sudo apt-get install -y

  rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

  catkin_make

  source devel/setup.bash
```

## I2C Connection
For I2C communication functions, I am using the i2c-tools and i2c-dev packages.

```
sudo apt-get install -y i2c-tools libi2c-dev
```
Now we can use the i2cdetect command to query the I2C bus. By default Adafruit sensor to the Raspberry Pi I2C bus, the sensor responds with the slave address 0x28. 
```
hostname@user:/home/pi# i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

## Launch files
```
roslaunch ros_bno055 bno055_imu.launch
```
Before launching, parameters in .launch file should be set accordingly.

## Parameters

* **device** -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** -- the i2c address of the IMU. Default is 0x28.
* **frame_id** -- frame_id for ROS tf. Default is "imu".
* **operation_mode** -- the i2c operation mode of the IMU. Default is imu.
* **power_mode** -- the i2c power mode of the IMU. Default is normal.
* **output_rate** -- Sensor data output rate of the IMU. Default is 30(Hz).
* **calib_rate** --  Self calibration rate of the IMU. Default is 1(Hz).
* **calib_timeout** -- Timeout for self calibration until it shutdowns the IMU. Default is 20 seconds.

## Outputs topics:
* **~/imu/data** (sensor\_msgs/Imu) -- fused IMU data
* **~/orientation_euler** (ros\_bno055/OrientationEuler) -- raw accelerometer data
* **~/magnetic_field** (sensor\_msgs/MagneticField) -- raw magnetic field data
* **~/temperature** (sensor\_msgs/Temperature) -- temperature data
* **~/gravity** (ros\_bno055/Gravity) -- gravity data

## Usage notes
Self calibration of the sensor currently only works for **imu, ndof and ndof_fmc** operation modes, therefore any other cannot be used. This is under progress and will be updated.

For more information checkout the sensor data sheet: 
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

