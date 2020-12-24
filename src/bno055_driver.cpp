#include "ros_bno055/bno055_driver.h"
#include <cstring>

bno055::Bno055Driver::Bno055Driver() : pow_mode_(NORMAL_MODE), opr_mode_(CONFIG_MODE)
{
  printf("BNO055 IMU driver initialized.\n");
}

// Method to initialize I2C connection with BNO055 sensor.
bool bno055::Bno055Driver::initI2c(const char* i2cBus, const __u8 i2cAddr)
{
  // Open I2C Bus.
  if ((file_desc_ = open(i2cBus, O_RDWR)) < 0)
  {
    printf("ERROR: Could not open I2C bus: %s.\n", i2cBus); 
    exit(-1);
  }
  else 
  {
    printf("Opened I2C bus: %s.\n", i2cBus);
  }

  // Locate BNO055 sensor
  if (ioctl(file_desc_, I2C_SLAVE, i2cAddr) < 0)
  {
    printf("ERROR: Could not locate BNO055 sensor at address: 0x%02X.\n", i2cAddr);
    exit(-1);
  }
  else 
  {
    printf("Located BNO055 sensor at address: 0x%02X.\n", i2cAddr);
  }

  return true;
}


bool bno055::Bno055Driver::reset() {
    opmode_t lastopmode = opr_mode_;
    pwrmode_t lastpwrmode = pow_mode_;	

    setConfigMode();

    // reset
    i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::SYS_TRIGGER, 0x20);
    usleep(250000);

    int i = 0;
    // wait for chip to come back online
    while(i2c_smbus_read_byte_data(file_desc_, bno055::RegisterMap::CHIP_ID) != BNO055_ID) {
    	usleep(1000);
        if(i++ > 500) {
            printf("chip did not come back online in 5 seconds after reset");
            return false;
        }
    }
    usleep(250000);

    // normal power mode
    i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::PWR_MODE, lastpwrmode);
    usleep(1000);

    i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::PAGE_ID, 0);
    i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::SYS_TRIGGER, 0);
    usleep(250000);

    i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, lastopmode);
    usleep(250000);

    return true;
}

// Method to get current power mode.
int bno055::Bno055Driver::getPowMode() {
  return pow_mode_;
}

// Method to get current operation mode.
int bno055::Bno055Driver::getOprMode() {
  return opr_mode_;
}

// Method to set/reset operation mode to CONFIG.
// Used for "resetting" the BNO055 sensor after certain read/write operations.
bool bno055::Bno055Driver::setConfigMode() {
  opmode_t newmode = CONFIG_MODE;
  if (getOprMode() != newmode) {
	  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, newmode) < 0)
	  {
	    printf("ERROR: Could not set operation mode to CONFIG.\n");
	    exit(-1);
	  }
	  else 
	  {
	    printf("Set operation mode to CONFIG: 0x%02X.\n", newmode);
	  } 
  }
  opr_mode_ = newmode;
  usleep(500000);

  return true;
}

bool bno055::Bno055Driver::setOperationMode(const char* opr_mode) {
   setConfigMode();
	
   opmode_t newmode;
   if (strlen(opr_mode) > 0) {
      if     (strcmp(opr_mode, "config")   == 0) newmode = CONFIG_MODE;
      else if(strcmp(opr_mode, "acconly")  == 0) newmode = ACC_ONLY;
      else if(strcmp(opr_mode, "magonly")  == 0) newmode = MAG_ONLY;
      else if(strcmp(opr_mode, "gyronly")  == 0) newmode = GYRO_ONLY;
      else if(strcmp(opr_mode, "accmag")   == 0) newmode = ACC_MAG;
      else if(strcmp(opr_mode, "accgyro")  == 0) newmode = ACC_GYRO;
      else if(strcmp(opr_mode, "maggyro")  == 0) newmode = MAG_GYRO;
      else if(strcmp(opr_mode, "amg")      == 0) newmode = AMG;
      else if(strcmp(opr_mode, "imu")      == 0) newmode = IMU;
      else if(strcmp(opr_mode, "compass")  == 0) newmode = COMPASS;
      else if(strcmp(opr_mode, "m4g")      == 0) newmode = M4G;
      else if(strcmp(opr_mode, "ndof")     == 0) newmode = NDOF;
      else if(strcmp(opr_mode, "ndof_fmc") == 0) newmode = NDOF_FMC_OFF;
      else {
         printf("Error: invalid operations mode %s.\n", opr_mode);
         return false;
      }
   }
   else {
      printf("Error: No Operation mode specified.\n");
      return false;
   }
   if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, newmode) < 0)
   {
     printf("ERROR: Could not set operation mode to IMU.\n");
     exit(-1);
   } 
   else 
   {
     printf("Set operation mode to: 0x%02X.\n", newmode);
   } 
   opr_mode_ = newmode;
   usleep(500000);

   return true;
}

bool bno055::Bno055Driver::getAcc() {
  bno055::AccData acc_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::ACC_DATA_X_LSB, 0x06, (__u8*)&acc_data) != 0x06) 
  {
    printf("ERROR: Could not read accelerometer data.\n");
    exit(-1);
  }
 
  // Convert LSB to m/s^2.
  bno055::Bno055Driver::data_.acc_x_ = (double)acc_data.acc_x / 100.0;
  bno055::Bno055Driver::data_.acc_y_ = (double)acc_data.acc_y / 100.0;
  bno055::Bno055Driver::data_.acc_z_ = (double)acc_data.acc_z / 100.0;

  return true;
}

bool bno055::Bno055Driver::getMag() { 
  bno055::MagData mag_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::MAG_DATA_X_LSB, 0x06, (__u8*)&mag_data) != 0x06) 
  {
    printf("ERROR: Could not read magnetometer data.\n");
    exit(-1);
  }
  
  // Convert LSB to uT (microtesla).
  bno055::Bno055Driver::data_.mag_x_ = (double)mag_data.mag_x / 16.0;
  bno055::Bno055Driver::data_.mag_y_ = (double)mag_data.mag_y / 16.0;
  bno055::Bno055Driver::data_.mag_z_ = (double)mag_data.mag_z / 16.0;

  return true;
}

bool bno055::Bno055Driver::getGyr() {
  bno055::GyrData gyr_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::GYR_DATA_X_LSB, 0x06, (__u8*)&gyr_data) != 0x06) 
  {
    printf("ERROR: Could not read gyroscope data.\n");
    exit(-1);
  }
  
  // Convert LSB to revolutions per second (rps).
  bno055::Bno055Driver::data_.gyr_x_ = (double)gyr_data.gyr_x / 900.0;
  bno055::Bno055Driver::data_.gyr_y_ = (double)gyr_data.gyr_y / 900.0;
  bno055::Bno055Driver::data_.gyr_z_ = (double)gyr_data.gyr_z / 900.0;

  return true;
}

bool bno055::Bno055Driver::getEul() {
  bno055::EulData eul_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::EUL_HEADING_LSB, 0x06, (__u8*)&eul_data) != 0x06)
  {
    printf("ERROR: Could not read euler angles data.\n");
    exit(-1);
  }
  
  // Convert LSB to degree.
  bno055::Bno055Driver::data_.eul_heading_ = (double)eul_data.eul_heading / 16.0;
  bno055::Bno055Driver::data_.eul_roll_ = (double)eul_data.eul_roll / 16.0;
  bno055::Bno055Driver::data_.eul_pitch_ = (double)eul_data.eul_pitch / 16.0;

  return true;
}

bool bno055::Bno055Driver::getQua() {
  bno055::QuaData qua_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::QUA_DATA_W_LSB, 0x08, (__u8*)&qua_data) != 0x08)
  {
    printf("ERROR: Could not read quaternions data.\n");
    exit(-1);
  }
  
  // Convert LSB to Quaternion.
  bno055::Bno055Driver::data_.qua_w_ = (double)qua_data.qua_w / 16384.0;
  bno055::Bno055Driver::data_.qua_x_ = (double)qua_data.qua_x / 16384.0;
  bno055::Bno055Driver::data_.qua_y_ = (double)qua_data.qua_y / 16384.0;
  bno055::Bno055Driver::data_.qua_z_ = (double)qua_data.qua_z / 16384.0;

  return true;
}

bool bno055::Bno055Driver::getLia() {
  bno055::LiaData lia_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::LIA_DATA_X_LSB, 0x06, (__u8*)&lia_data) != 0x06) 
  {
    printf("ERROR: Could not read linear acceleration data.\n");
    exit(-1);
  }
 
  // Convert LSB to m/s^2.
  bno055::Bno055Driver::data_.lia_x_ = (double)lia_data.lia_x / 100.0;
  bno055::Bno055Driver::data_.lia_y_ = (double)lia_data.lia_y / 100.0;
  bno055::Bno055Driver::data_.lia_z_ = (double)lia_data.lia_z / 100.0;

  return true;
}

bool bno055::Bno055Driver::getGrv() {
  bno055::GrvData grv_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::GRV_DATA_X_LSB, 0x06, (__u8*)&grv_data) != 0x06) 
  {
    printf("ERROR: Could not read gravity vector data.\n");
    exit(-1);
  }
 
  // Convert LSB to m/s^2.
  bno055::Bno055Driver::data_.grv_x_ = (double)grv_data.grv_x / 100.0;
  bno055::Bno055Driver::data_.grv_y_ = (double)grv_data.grv_y / 100.0;
  bno055::Bno055Driver::data_.grv_z_ = (double)grv_data.grv_z / 100.0;

  return true;
}

bool bno055::Bno055Driver::getTemp() {
  bno055::TempData temp_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::TEMP, 0x01, (__u8*)&temp_data) != 0x01) 
  {
    printf("ERROR: Could not read temperature data.\n");
    exit(-1);
  }
 
  bno055::Bno055Driver::data_.temp_ = (double)temp_data.temp;

  return true;
}

bool bno055::Bno055Driver::getCalibStat() {
  bno055::CalibStatData calib_stat_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::CALIB_STAT, 0x01, (__u8*)&calib_stat_data) != 0x01) 
  {
    printf("ERROR: Could not read calibration status data.\n");
    exit(-1);
  }
 
  // Bitwise AND to clear (reset to 0) irrelevant bits.
  bno055::Bno055Driver::data_.calib_stat_sys_ = (calib_stat_data.calib_stat & 0b11000000) >> 6;
  bno055::Bno055Driver::data_.calib_stat_gyr_ = (calib_stat_data.calib_stat & 0b00110000) >> 4;
  bno055::Bno055Driver::data_.calib_stat_acc_ = (calib_stat_data.calib_stat & 0b00001100) >> 2;
  bno055::Bno055Driver::data_.calib_stat_mag_ = (calib_stat_data.calib_stat & 0b00000011);

  printf("System Calibration Status: %d.\n", bno055::Bno055Driver::data_.calib_stat_sys_);
  printf("Accelerometer Calibration Status: %d.\n", bno055::Bno055Driver::data_.calib_stat_acc_);
  printf("Magnetometer Calibration Status: %d.\n", bno055::Bno055Driver::data_.calib_stat_mag_);
  printf("Gyroscope Calibration Status: %d.\n", bno055::Bno055Driver::data_.calib_stat_gyr_);

  return true;
}

bool bno055::Bno055Driver::getCalibOffset() {
  opmode_t lastmode = opr_mode_;

  setConfigMode();

  bno055::CalibOffsetData calib_offset_data;

  // Read 18 bytes into offset data structure (6 bytes for acc, 6 bytes for mag, 6 bytes for gyr).
  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::ACC_OFFSET_X_LSB, 0x12, (__u8*)&calib_offset_data) != 0x12) 
  {
    printf("ERROR: Could not read calibration offset data.\n");
    exit(-1);
  }
 
  bno055::Bno055Driver::data_.acc_offset_x_ = calib_offset_data.acc_offset_x;
  bno055::Bno055Driver::data_.acc_offset_y_ = calib_offset_data.acc_offset_y;
  bno055::Bno055Driver::data_.acc_offset_z_ = calib_offset_data.acc_offset_z;
  bno055::Bno055Driver::data_.mag_offset_x_ = calib_offset_data.mag_offset_x;
  bno055::Bno055Driver::data_.mag_offset_y_ = calib_offset_data.mag_offset_y;
  bno055::Bno055Driver::data_.mag_offset_z_ = calib_offset_data.mag_offset_z;
  bno055::Bno055Driver::data_.gyr_offset_x_ = calib_offset_data.gyr_offset_x;
  bno055::Bno055Driver::data_.gyr_offset_y_ = calib_offset_data.gyr_offset_y;
  bno055::Bno055Driver::data_.gyr_offset_z_ = calib_offset_data.gyr_offset_z;

  printf("Accelerometer Offset: X: %d, Y: %d, Z: %d.\n", bno055::Bno055Driver::data_.acc_offset_x_, bno055::Bno055Driver::data_.acc_offset_y_, bno055::Bno055Driver::data_.acc_offset_z_);  
  printf("Magnetometer Offset: X: %d, Y: %d, Z: %d.\n", bno055::Bno055Driver::data_.mag_offset_x_, bno055::Bno055Driver::data_.mag_offset_y_, bno055::Bno055Driver::data_.mag_offset_z_);  
  printf("Gyroscope Offset: X: %d, Y: %d, Z: %d.\n", bno055::Bno055Driver::data_.gyr_offset_x_, bno055::Bno055Driver::data_.gyr_offset_y_, bno055::Bno055Driver::data_.gyr_offset_z_);  

  // Reset to previous mode.
  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, lastmode) < 0)
  {
    printf("ERROR: Could not set operation mode to previous mode.\n");
    exit(-1);
  } 
  else 
  {
    printf("Set operation mode to previous mode: 0x%02X.\n", lastmode);
  } 
  opr_mode_ = lastmode;
  usleep(500000);

  return true;
}

bool bno055::Bno055Driver::getCalibRadius() {
  opmode_t lastmode = opr_mode_;

  setConfigMode();

  bno055::CalibRadiusData calib_radius_data;

  // Read 4 bytes into offset data structure (2 bytes for acc, 2 bytes for mag).
  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::ACC_RADIUS_LSB, 0x04, (__u8*)&calib_radius_data) != 0x04) 
  {
    printf("ERROR: Could not read calibration radius data.\n");
    exit(-1);
  }
 
  bno055::Bno055Driver::data_.acc_radius_ = calib_radius_data.acc_radius;
  bno055::Bno055Driver::data_.mag_radius_ = calib_radius_data.mag_radius;

  printf("Accelerometer Radius: %d.\n", bno055::Bno055Driver::data_.acc_radius_);  
  printf("Magnetometer Radius: %d.\n", bno055::Bno055Driver::data_.mag_radius_);  

  // Reset to previous mode.
  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, lastmode) < 0)
  {
    printf("ERROR: Could not set operation mode to previous mode.\n");
    exit(-1);
  } 
  else 
  {
    printf("Set operation mode to previous mode: 0x%02X.\n", lastmode);
  } 
  opr_mode_ = lastmode;
  usleep(500000);

  return true;
}

bool bno055::Bno055Driver::loadCalib() {
  opmode_t lastmode = opr_mode_;

  setConfigMode();

  // Predetermined offsets and radii values - Should be tuned!
  __u16 acc_offset[3] = {65527, 65527, 0};
  __u16 mag_offset[3] = {196, 65521, 64968};
  __u16 gyr_offset[3] = {65534, 65534, 1};
  __u16 acc_radius[1] = {1000};
  __u16 mag_radius[1] = {805};
  
  // Write offsets and radii values.
  i2c_smbus_write_i2c_block_data(file_desc_, RegisterMap::ACC_OFFSET_X_LSB, 0x06, (__u8*)&acc_offset[0]);
  i2c_smbus_write_i2c_block_data(file_desc_, RegisterMap::MAG_OFFSET_X_LSB, 0x06, (__u8*)&mag_offset[0]);
  i2c_smbus_write_i2c_block_data(file_desc_, RegisterMap::GYR_OFFSET_X_LSB, 0x06, (__u8*)&gyr_offset[0]);
  i2c_smbus_write_i2c_block_data(file_desc_, RegisterMap::ACC_RADIUS_LSB, 0x02, (__u8*)&acc_radius[0]);
  i2c_smbus_write_i2c_block_data(file_desc_, RegisterMap::MAG_RADIUS_LSB, 0x02, (__u8*)&mag_radius[0]);
  
  printf("Setting Accelerometer Offset: X: %d, Y: %d, Z: %d.\n", acc_offset[0], acc_offset[1], acc_offset[2]);  
  printf("Setting Magnetometer Offset: X: %d, Y: %d, Z: %d.\n", mag_offset[0], mag_offset[1], mag_offset[2]);  
  printf("Setting Gyroscope Offset: X: %d, Y: %d, Z: %d.\n", gyr_offset[0], gyr_offset[1], gyr_offset[2]);  
  printf("Setting Accelerometer Radius: %d.\n", acc_radius[0]);  
  printf("Setting Magnetometer Radius: %d.\n", mag_radius[0]);  
 
  // Reset to previous mode.
   if (getOprMode() != lastmode) {
	  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, lastmode) < 0) {
	    printf("ERROR: Could not set operation mode to previous mode.\n");
	    exit(-1);
	  } 
	  else printf("Set operation mode to previous mode: 0x%02X.\n", lastmode);
   }
  opr_mode_ = lastmode;
  usleep(500000);

  return true;
}

bno055::Bno055Driver::~Bno055Driver()
{
  if (!reset()) printf("Could not reset the device.");
  printf("BNO055 IMU driver reseted and destroyed.\n");  
}
