package newinventions;

import android.util.Log;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.concurrent.locks.Lock;

/**
 * Created by Owner on 8/25/2015.
 * Original author: Alan M. Gilkes, Alternate Coach of FTC Team 3734 ("imperial Robotics")
 * and Mentor of FTC Team 6832 ("Iron Reign")
 * Contact: Email - agilkes@alum.mit.edu, FTC Technology Forum ID - AlanG
 *
 * Overview:
 *
 * This class uses the FTC SDK to provide a software interface to the BNO055 9-DOF Inertial Measurement
 * Unit (IMU) manufactured by Bosch and sold by Adafruit:
 * https://www.adafruit.com/products/2472 ($34.95, as of 25 August 2015)
 * This software assumes that the IMU is connected via a 4-wire cable to one of the I2C ports on the
 * Modern Robotics (MR) Core Device Interface Module (CDIM). (Note: There is a difference between
 * the 4-wire pinout of the I2C ports and the pinout on the IMU board. Crossovers ARE REQUIRED in the
 * 4-wire cable.
 *
 * The datasheet at http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf explains the IMU's
 * functions and interfaces in detail. Also, this software borrows heavily from the open source sensor
 * driver code located on GitHub at https://github.com/adafruit/Adafruit_BNO055
 * The relevant C++ and header files are: Adafruit_BNO055.cpp and Adafruit_BNO055.h
 *
 * Initial interest in the IMU is as a substitute for a heading (yaw angle) gyro. In its fusion mode,
 * the IMU's built-in microcontroller numerically integrates the rates sampled from its 3 built-in
 * gyros and reports the integration results as Euler angles for roll, pitch and heading.
 * (For an explanation of Euler angles, see https://en.wikipedia.org/wiki/Euler_angles)
 *
 * Revision history:
 *
 * 11 September 2015:
 *  Code to instantiate the class, initialize the IMU, and return a heading angle, when requested (AMG)
 *
 */

public class AdafruitIMU implements HardwareDevice, I2cController.I2cPortReadyCallback{

  public static final int BNO055_ADDRESS_A = 0x28;//From Adafruit_BNO055.h
  public static final int BNO055_ADDRESS_B = 0x29;
  public static final int BNO055_ID        = 0xA0;
  public static final int                         //From Adafruit_BNO055.h
      /* Page id register definition */
    BNO055_PAGE_ID_ADDR                                     = 0X07,

  /* PAGE0 REGISTER DEFINITION START*/
  BNO055_CHIP_ID_ADDR                                     = 0x00,
    BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
    BNO055_MAG_REV_ID_ADDR                                  = 0x02,
    BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
    BNO055_BL_REV_ID_ADDR                                   = 0X06,

  /* Accel data register */
  BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
    BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

  /* Mag data register */
  BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
    BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

  /* Gyro data registers */
  BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
    BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,
  /* For IMU mode, the register addresses 0X1A thru 0X2D (20 bytes) should be read consecutively */
  /* Euler data registers */
  BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
    BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
    BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
    BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
    BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
    BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

  /* Quaternion data registers */
  BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

  /* Linear acceleration data registers */
  BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

  /* Gravity data registers */
  BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

  /* Temperature data register */
  BNO055_TEMP_ADDR                                        = 0X34,

  /* Status registers */
  BNO055_CALIB_STAT_ADDR                                  = 0X35,
    BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
    BNO055_INTR_STAT_ADDR                                   = 0X37,

  BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
    BNO055_SYS_STAT_ADDR                                    = 0X39,
    BNO055_SYS_ERR_ADDR                                     = 0X3A,

  /* Unit selection register */
  BNO055_UNIT_SEL_ADDR                                    = 0X3B,
    BNO055_DATA_SELECT_ADDR                                 = 0X3C,

  /* Mode registers */
  BNO055_OPR_MODE_ADDR                                    = 0X3D,
    BNO055_PWR_MODE_ADDR                                    = 0X3E,

  BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
    BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

  /* Axis remap registers */
  BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

  /* SIC registers */
  BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

  /* Accelerometer Offset registers */
  ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

  /* Magnetometer Offset registers */
  MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
    MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

  /* Gyroscope Offset register s*/
  GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
    GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

  /* Radius registers */
  ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
    ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
    MAG_RADIUS_LSB_ADDR                                     = 0X69,
    MAG_RADIUS_MSB_ADDR                                     = 0X6A;
  public static final int                         //From Adafruit_BNO055.h
    POWER_MODE_NORMAL                                       = 0X00,
    POWER_MODE_LOWPOWER                                     = 0X01,
    POWER_MODE_SUSPEND                                      = 0X02;
  public static final int                         //From Adafruit_BNO055.h
      /* Operation mode settings*/
    OPERATION_MODE_CONFIG                                   = 0X00,
    OPERATION_MODE_ACCONLY                                  = 0X01,
    OPERATION_MODE_MAGONLY                                  = 0X02,
    OPERATION_MODE_GYRONLY                                  = 0X03,
    OPERATION_MODE_ACCMAG                                   = 0X04,
    OPERATION_MODE_ACCGYRO                                  = 0X05,
    OPERATION_MODE_MAGGYRO                                  = 0X06,
    OPERATION_MODE_AMG                                      = 0X07,
    OPERATION_MODE_IMU                                      = 0X08, //Added to original C++ list
    OPERATION_MODE_IMUPLUS                                  = 0X08,
    OPERATION_MODE_COMPASS                                  = 0X09,
    OPERATION_MODE_M4G                                      = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
    OPERATION_MODE_NDOF                                     = 0X0C;

  private final int i2cBufferSize = 26; //Size of any extra buffers that will hold any incoming or
                                        // outgoing cache data
  private final I2cDevice i2cIMU; //The device class of the Adafruit/Bosch IMU
  //The I2cDevice class does not work properly in the 3 August 2015 release of the FTC SDK<=====***********
  private final DeviceInterfaceModule deviceInterface;//The Core Device Interface Module that the IMU
                                                      //is plugged into
  private final int configuredI2CPort;//The I2C port on the Core Device Interface Module that the IMU
                                      //is plugged into
  private final int baseI2Caddress; //The base I2C address used to address all of the IMU's registers
  private int operationalMode;//The operational mode to which the IMU will be set after its initial
                              //reset.
  private final byte[] i2cReadCache;//The interface will insert the bytes which have been read into
                                    // this cache
  private final byte[] i2cWriteCache; //This cache will hold the bytes which are to be written to
                                      //the interface
  private final Lock i2cReadCacheLock;//A lock on access to the IMU's I2C read cache
  private final Lock i2cWriteCacheLock; //A lock on access to the IMU's I2C write cache
  /* For IMU mode, the register addresses 0X1A thru 0X2D (20 bytes) should be read consecutively */
  /* Enable I2C Read Mode and address the bytes in the ReadCache using the following parameters: */
  private int numberOfRegisters = 20;
  private int registerStartAddress = BNO055_EULER_H_LSB_ADDR;
  private int readCacheOffset = BNO055_EULER_H_LSB_ADDR - I2cController.I2C_BUFFER_START_ADDRESS;
  /* The folowing variables instrument the performance of I2C reading and writing */
  public long totalI2Creads;
  public double maxReadInterval;
  public double avgReadInterval;
  private long readStartTime;

  private void snooze(long milliSecs){//Simple utility for sleeping (thereby releasing the CPU to
                                      // threads other than this one)
    try {
      Thread.sleep(milliSecs);
    } catch (InterruptedException e){}
  }
    /*
     * Operational modes are explained in the IMU datasheet in Table 3-3 on p.20, and elsewhere
     * in Section 3.3 which begins on p.20. A "fusion" mode must be chosen, in order for the IMU to
     * produce numerically integrated Euler angles from the gyros. Since FTC robotics activities
     * typically occur in environments that interfere with magnetic compasses, a "fusion" mode like
     * "IMU" (a.k.a "IMUPLUS") is an appropriate choice, It runs only the accelerometers and the
     * gyros, and the Euler angles it generates for heading, pitch and yaw are relative to the
     * robot's initial orientation, not absolute angles relative to the inertial reference frame of
     * the earth's magnetic north and the earth's gravity.
    */

  //The Constructor for the AdafruitIMU class
  public AdafruitIMU (HardwareMap currentHWmap, String configuredIMUname,
                      String configuredInterfaceName, int configuredPort,
                      byte baseAddress, byte operMode) throws RobotCoreException{
    long calibrationStartTime;
    byte[] outboundBytes = new byte[i2cBufferSize];
    //i2cIMU = currentHWmap.i2cDevice.get(configuredIMUname); //Not recommended for the 8 Aug 15 release
    deviceInterface = currentHWmap.deviceInterfaceModule.get(configuredInterfaceName);
    Log.i("FtcRobotController", "Core Device Interface Module info: "
           + deviceInterface.getConnectionInfo());
    configuredI2CPort = configuredPort;
    i2cIMU = new I2cDevice(deviceInterface, configuredI2CPort); //Identify the IMU with the port to
            //which it is connected on the Modern Robotics Core Device Interface Module
    baseI2Caddress = (int)baseAddress & 0XFF;
    operationalMode = (int)operMode & 0XFF;
    i2cReadCache = i2cIMU.getI2cReadCache();
    i2cReadCacheLock = i2cIMU.getI2cReadCacheLock();
    i2cWriteCache = i2cIMU.getI2cWriteCache();
    i2cWriteCacheLock = i2cIMU.getI2cWriteCacheLock();
    //Log.i("FtcRobotController", String.format("Read Cache length: %d, Write Cache length: %d."
    //      , i2cReadCache.length, i2cWriteCache.length));
    /*
     * According to the IMU datasheet, the defaults set up at power-on reset:
     * PWR_MODE register = normal (p.19, table 3-1)
     * OPR_MODE register = CONFIGMODE (p. 21, table 3-5)
     * Default configuration for sensors as listed in table 3-7 on p.26
     * Page 0 of the register map is automatically selected (p.50)
     *
     * Even if the Robot Controller software is restarted without power being cycled, the following
     * actions should be done each time the AdafruitIMU class is reconstructed:
     * 1. Set the PAGE_ID register to 0, so that Register Map 0 will make the SYS_TRIGGER register
     *    visible. (Datasheet p.50)
     * 2. Reset the IMU, which causes output values to be reset to zero, and forces accelerometers,
     *    gyros, and magnetic compasses to autocalibrate. (See IMU data sheet p. 18, Table 4-2, p.51
     *    , and Section 3.10, p.47. Also, set the bit that commands self-test.
     * 3. OPR_MODE register = the user-selected operationalMode (passed in as operMode)
    */
    Log.i("FtcRobotController", "Resetting IMU to its power-on state......");
    //Set the register map PAGE_ID back to 0, to make the SYS_TRIGGER register visible
    outboundBytes[0] = 0x00;//Sets the PAGE_ID bit for page 0 (Table 4-2)
    if (i2cWriteImmediately(outboundBytes, 1, BNO055_PAGE_ID_ADDR)) {
      //Set the RST_SYS bit in the SYS_TRIGGER register, to make the IMU reset

      //outboundBytes[0] = 0x61;//The "6" sets the RST_SYS and RST_INT bits, and clears the CLK_SEL bit
      //, to enable the IMU's internal clock (Table 4-2, and p.70). In the lower 4 bits, a "1" sets
      // the commanded Self Test bit, which causes self-test to run (p. 46)

      /*
      * Houston, we have a problem! As of 11 September 2015, the FTC API and the firmware in the
      * Modern Robotics Core Device Interface Module fail at any attempt to reset the IMU, or
      * command it to do its self-test
      */
      outboundBytes[0] = 0x00;//Temporarily, write a "NO-OP" into the SYS_TRIGGER register

      if (i2cWriteImmediately(outboundBytes, 1, BNO055_SYS_TRIGGER_ADDR)) {
        Log.i("FtcRobotController", "Resetting the IMU........");
        snooze(500);//Wait a decent interval until the IMU finishes its reset operations
        Log.i("FtcRobotController", "Now waiting for autocalibration............");
        calibrationStartTime = System.nanoTime();
        if (autoCalibrationOK(10)) {//Check auto calibration with a timeout (seconds) on the wait
                                    //for the I2C port to become ready
          Log.i("FtcRobotController", "Auto calibration completed OK after "
            + String.format("%3.3f", (double)((System.nanoTime() - calibrationStartTime) / 1000000000L))
                              + " sec.");
          //And finally, set the IMU's operational mode to the selected mode
          outboundBytes[0] = (byte)operationalMode;
          if (i2cWriteImmediately(outboundBytes, 1, BNO055_OPR_MODE_ADDR)) {
            Log.i("FtcRobotController", "IMU fully operational!");
          } else {
            throw new RobotCoreException("Operational mode setting interrupted or I2C bus" +
                                           " \"stuck busy\".");
          }
        } else {
          throw new RobotCoreException("Auto calibration interrupted or I2C bus \"stuck busy\", " +
            "OR it timed out after "
            + String.format("%3.3f", (double)((System.nanoTime() - calibrationStartTime) / 1000000000L))
            + " sec.");

        }
      } else {
        throw new RobotCoreException("IMU reset interrupted or I2C bus \"stuck busy\".");
      }
    } else {
      throw new RobotCoreException("IMU PAGE_ID setting interrupted or I2C bus \"stuck busy\".");
    }
  }

  private boolean i2cWriteImmediately(byte[]outboundBytes, int byteCount, int registerAddress){
    long rightNow = System.nanoTime(), startTime = System.nanoTime(), timeOut = 3000000000L;
    int indx;

    try {
      while ((!i2cIMU.isI2cPortReady())
               && (((rightNow = System.nanoTime()) - startTime) < timeOut)){
        Thread.sleep(250);//"Snooze" right here, until the port is ready (a good thing) OR n billion
        //nanoseconds pass with the port "stuck busy" (a VERY bad thing)
      }
    } catch (InterruptedException e) {
      Log.i("FtcRobotController", "Unexpected interrupt while \"sleeping\" in i2cWriteImmediately.");
      return false;
    }
    if ((rightNow - startTime) >= timeOut) return false;//Signals the "stuck busy" condition
    try {
      i2cWriteCacheLock.lock();
      for (indx =0; indx < byteCount; indx++) {
        i2cWriteCache[I2cController.I2C_BUFFER_START_ADDRESS + indx] = outboundBytes[indx];
        //Both the read and write caches start with 5 bytes of prefix data.
      }
    } finally {
      i2cWriteCacheLock.unlock();
    }
    //The device interface object must do this, because the i2c device object CAN'T do it, in the
    //8 August 2015 beta release of the FTC SDK
    deviceInterface.enableI2cWriteMode(configuredI2CPort, baseI2Caddress, registerAddress, byteCount);
    i2cIMU.setI2cPortActionFlag();  //Set the "go do it" flag
    i2cIMU.writeI2cCacheToModule(); //Then write it and the cache out
    snooze(250);//Give the data time to go from the Interface Module to the IMU hardware
    return true;
  }

  private boolean autoCalibrationOK(int timeOutSeconds){
    boolean readingEnabled = false, calibrationDone = false;
    long calibrationStart = System.nanoTime(), rightNow = System.nanoTime(),
      loopStart = System.nanoTime();;

    while ((System.nanoTime() - calibrationStart) <= 60000000000L) {//Set a 1-minute overall timeout
      try {
        loopStart = System.nanoTime();
        while ((!i2cIMU.isI2cPortReady())
                 && (((rightNow = System.nanoTime()) - loopStart) < (timeOutSeconds * 1000000000L))) {
          Thread.sleep(250);//"Snooze" right here, until the port is ready (a good thing) OR n billion
          //nanoseconds pass with the port "stuck busy" (a VERY bad thing)
        }
      } catch (InterruptedException e) {
        Log.i("FtcRobotController", "Unexpected interrupt while \"sleeping\" in autoCalibrationOK.");
        return false;
      }
      if ((rightNow - loopStart) >= (timeOutSeconds * 1000000000L)) {
        Log.i("FtcRobotController", "IMU I2C port \"stuck busy\" for "
          + (rightNow - loopStart) + " ns.");
        return false;//Signals the "stuck busy" condition
      }
      if (!readingEnabled) {//Start a stream of reads of the calibration status byte
        //The device interface object must do this, because the i2c device object CAN'T do it, in the
        //8 August 2015 beta release of the FTC SDK
        deviceInterface.enableI2cReadMode(configuredI2CPort, baseI2Caddress,
                                           BNO055_CALIB_STAT_ADDR, 2);
        //deviceInterface.enableI2cReadMode(configuredI2CPort, baseI2Caddress,//FOR TESTING ONLY!
        //                                   BNO055_CHIP_ID_ADDR, 1);         //FOR TESTING ONLY!
        //deviceInterface.enableI2cReadMode(configuredI2CPort, baseI2Caddress,//FOR TESTING ONLY!
        //                                   BNO055_PAGE_ID_ADDR, 1);         //FOR TESTING ONLY!
        i2cIMU.setI2cPortActionFlag();//Set this flag to do the next read
        i2cIMU.writeI2cCacheToModule();
        snooze(250);//Give the data time to go from the Interface Module to the IMU hardware
        readingEnabled = true;
        continue;
      } else {//Check the Calibration Status and Self-Test bytes in the Read Cache. IMU datasheet
        // Sec. 3.10, p.47. Also, see p. 70
        i2cIMU.readI2cCacheFromModule();//Read in the most recent data from the device
        snooze(500);//Give the data time to come into the Interface Module from the IMU hardware
        try {
          i2cReadCacheLock.lock();
          if (((i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS + 1] & 0X0C) >= (byte)0X0C) &&
            ((i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS] & 0XFF) >= (byte)0X00) //"NO-OP"
            //(i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS] == (byte)BNO055_ID)//FOR TESTING ONLY!
            //(i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS] == (byte)0X00)//FOR TESTING ONLY!
            ) {
            //See IMU datasheet p.67. As an example, 0X30 checks only gyro calibration complete.
            //Also on that page: Self-Test byte value of 0X0C means MCU and gyro passed self-tests
            Log.i("FtcRobotController", "Autocalibration OK! Cal status byte = "
              + String.format("0X%02X", i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS])
              + ". Self Test byte = "
              + String.format("0X%02X", i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS + 1])
              + ".");
            calibrationDone = true;//Auto calibration finished successfully
          }
        } finally {
          i2cReadCacheLock.unlock();
        }
        if(calibrationDone) return true;
        i2cIMU.setI2cPortActionFlag();   //Set this flag to do the next read
        i2cIMU.writeI2cPortFlagOnlyToModule();
        //At this point, the port becomes busy (not ready) doing the next read
        snooze(250);//Give the data time to go from the Interface Module to the IMU hardware
      }
    }
    Log.i("FtcRobotController", "Autocalibration timed out! Cal status byte = "
      + String.format("0X%02X",i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS])
      + ". Self Test byte = "
      + String.format("0X%02X",i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS + 1])
      + ".");
    return false;
  }

  public void startIMU(){
  /*
   * The IMU datasheet lists the following actions as necessary to initialize and set up the IMU,
   * asssuming that all operations of the Constructor completed successfully:
   * 1. Register the callback method which will keep the reading of IMU registers going
   * 2. Enable I2C read mode and start the self-perpetuating sequence of register readings
   *
  */
    i2cIMU.registerForI2cPortReadyCallback(this);
    //The device interface object must do this, because the i2c device object CAN'T do it, in the
    //8 August 2015 beta release of the FTC SDK
    deviceInterface.enableI2cReadMode(configuredI2CPort, baseI2Caddress, registerStartAddress
                                       , numberOfRegisters);
    maxReadInterval = 0.0;
    avgReadInterval = 0.0;
    totalI2Creads = 0L;
    i2cIMU.setI2cPortActionFlag();//Set this flag to do the next read
    i2cIMU.writeI2cCacheToModule();
    readStartTime = System.nanoTime();
  }

  public int getIMUHeadingAngle() {
  /*
   * The IMU reports a 16-bit heading angle between 0 and 360 degrees, increasing with clockwise
   * turns. Ref: Table 3-7, p.26, Table 3-13, p.30, and Section 3.6.5.4, p.35
   * This is a fixed-point number in degrees * 16, i.e., 12 integer bits and 4 fractional bits
  */
    int tempResult = 0;
    try {
      i2cReadCacheLock.lock();
      //tempResult = (int)(i2cReadCache[BNO055_EULER_H_MSB_ADDR - readCacheOffset] << 8)
      //               + (int)i2cReadCache[BNO055_EULER_H_LSB_ADDR - readCacheOffset];
      tempResult = (int)i2cReadCache[BNO055_EULER_H_MSB_ADDR - readCacheOffset] * 256
                     + (int)i2cReadCache[BNO055_EULER_H_LSB_ADDR - readCacheOffset];
    } finally {
      i2cReadCacheLock.unlock();
    }
    return tempResult;
  }

  /*
   * Use of the following callback assumes that I2C reading has been enabled for a particular I2C
   * register address (as the starting address) and a particular byte count. Registration of this
   * callback should only take place when that reading enablement is done.
  */
  public void portIsReady(int port) { //Implementation of I2cController.I2cPortReadyCallback
    double latestInterval;
    if ((latestInterval = ((System.nanoTime() - readStartTime) / 1000000.0)) > maxReadInterval)
      maxReadInterval = latestInterval;
    avgReadInterval = ((avgReadInterval * 511.0) + latestInterval)/512.0;
    i2cIMU.readI2cCacheFromModule(); //Read in the most recent data from the device
    totalI2Creads++;
    i2cIMU.setI2cPortActionFlag();   //Set this flag to do the next read
    i2cIMU.writeI2cPortFlagOnlyToModule();
    readStartTime = System.nanoTime();
    totalI2Creads++;
    //At this point, the port becomes busy (not ready) doing the next read
  }

  //All of the following implement the HardwareDevice Interface

  public String getDeviceName() {
    return "Bosch 9-DOF IMU BNO055";
  }

  public String getConnectionInfo() {
    //return this.b.getConnectionInfo() + "; I2C port " + this.c;
    //Temporarily:
    return ("IMU connection info??");
  }

  public int getVersion() {
    return BNO055_ID;
  } //Temporarily

  public void close() {
  }

}
