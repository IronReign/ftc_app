package com.qualcomm.ftcrobotcontroller.opmodes;

  import android.util.Log;

  import com.qualcomm.robotcore.eventloop.opmode.OpMode;
  import com.qualcomm.robotcore.exception.RobotCoreException;
  import com.qualcomm.robotcore.hardware.DcMotor;
  import com.qualcomm.robotcore.util.Range;

  import com.ironreign.FTCcommunity.AdafruitIMU;

/**
 * An example of an autonomous OpMode, showing a proposed skeleton for the design of such an OpMode.
 * Specifically, it shows how "worker" threads may cooperate with the OpMode's "loop" method to
 * do autonomous processing of DC motors and their encoders. This design is easily extended to
 * create an autonomous mode in which a robot drives a certain distance while SIMULTANEOUSLY
 * hoisting its manipulator (whatever that may be - for example, a ball delivery mechanism in the
 * 2014-2015 FTC Cascade Effect game.)
 */
public class TrialAutonomousOp extends OpMode {

  enum ThreadState {//To support implementation of state machines using "switch" statements, this
    //"enum" gives names to the "cases" included in the "switch" statements. The
    // names may, of course, be whatever the programmers consider meaningful.
    STATE_ZERO,
    STATE_ONE,
    STATE_TWO,
    STATE_THREE
  }
  /************************************************************************************************
   * The following are variables generally relevant to multiple methods in this OpMode (Constructor,
   * "worker" thread "run" methods, and the OpMode's start, loop, and stop, plus init (as of 8/6/2015).
   */
  DcMotor motorRight;
  DcMotor motorLeft;
  DcMotor motorHoist;
  AdafruitIMU boschBNO055;
  int motorRightEncoderOffset;
  int motorLeftEncoderOffset;
  int motorHoistEncoderOffset;
  Thread autoDriver;//"Worker thread" that guides the autonomous driving of the robot
  Thread autoHoist;//"Worker" thread that guides the autonomous movement of the hoist motor (and
  //any appendages attached to it)
  /*
  * The following are variables used specifically for interthread communication between the
  * autonomous "worker" threads and the EventLoopManager that runs this OpMode's loop method.
  * Each of these variables is written by one and only one thread, but may be read by one or more.
  * NOTE: AT PRESENT, IT IS ASSUMED THAT MUTUALLY EXCLUSIVE ACCESS TO THESE VARIABLES IS NOT
  * NECESSARY. If, in future, these variable need to be read or written in groups of 2 or more, then
  * Java's "synchronized" methods or code blocks must be used.
  */
  volatile float rightMotorPower;
  volatile float leftMotorPower;
  volatile float hoistMotorPower;
  volatile int motorRightCurrentEncoder;
  volatile int motorLeftCurrentEncoder;
  volatile int motorHoistCurrentEncoder;
  volatile int headingAngle;//Heading(yaw) Euler angle as reported by the IMU
  /*By making the state variables visible to all threads, each "worker" thread can read and write
  *its own state variable, and make it readable by all the other threads. This facilitates the
  *various threads' efforts to synchronize their activities.
  */
  volatile ThreadState hoistThreadState;
  volatile ThreadState driverThreadState;


  /************************************************************************************************
   * Constructor
   *
   * As of 7/22/15, analysis of FTC's "beta" SDK indicates the following:
   * After the FTC Driver Station selects this OpMode, the "private void b()" method of the
   * "OpModeManager" uses "newInstance()" to instantiate this OpMode, at which time this
   * Constructor is called. It must remain "nullary", i.e., free of input parameters, but it can
   * perform initializations that are appropriate before this OpMode's "start" method gets
   * called. That call is accomplished via the "startActiveOpMode" call later in the
   * OpMode Manager's method "b".
   * NOTE: IF THE "newInstance()" CALL DOES NOT RETURN UNTIL THIS CONSTRUCTOR COMPLETES, THEN THIS
   * CONSTRUCTOR MAY BE ABLE TO DO SUCH TASKS AS GYRO INITIALIZATION AND BIAS OFFSET
   * MEASUREMENT, BEFORE THE "start" METHOD EVER GETS CALLED!
   */
  public TrialAutonomousOp() {
    //Is this code best executed here, or in the init() method, which is new, as of 3 August 2015?
    //autoDriver = new Thread(new DriverThread());
    //autoHoist = new Thread(new HoistThread());
  }


  /*
  ********************************************************************************************
  * The following inner classes are the "worker" Threads which do the multitasking required by
  * FTC Autonomous mode. These are "plain vanilla" Java Threads. The Android OS's AsyncTask and
  * Service classes are not required.
  ********************************************************************************************
  */
  class DriverThread implements Runnable {
    long driverThreadInterval = 100;//Number of milliseconds between the starts of successive
    //iterations
    long iterationStart;
    DriverThread(){ //Constructor
      driverThreadState = ThreadState.STATE_ZERO; //A state machine must have an initial state
    }
    public void run() {
      while (!Thread.interrupted()) { //A call to "interrupt()" is what stops this thread
        iterationStart = System.currentTimeMillis();
          /*
          * See the text of NxtEncoderOp.java for a simple example of the use of a "switch"
          * statement to implement a state machine. The names of each state and the contents of
          * the corresponding "case" blocks may be whatever the programmers deem necessary/
          */
        switch (driverThreadState) {
          case STATE_ZERO: {
              /*
              * In this skeletal example, the code for this state might control how far the robot
              * drives off a ramp to get to a particular position on the playing field. Iteration
              * by iteration, this code can use successive values of
              * (motorRightCurrentEncoder-motorRightEncoderOffset) and
              * (motorLeftCurrentEncoder-motorLeftEncoderOffset) to generate successive values of
              * rightMotorPower and leftMotorPower. When this state's destination has been reached,
              * it can set driverThreadState to point to a successor state, which might, for
              * example, execute a turn.
              */
            break;
          }
          default: {
            break;
          }
        }
        try { //When the work of each iteration is done, "sleep" until time to start the next one.
          //The "sleep" time required depends on the duration of the iteration just ended.
          Thread.sleep(driverThreadInterval + Math.max(-driverThreadInterval,
                                                        (iterationStart - System.currentTimeMillis())));
        } catch (InterruptedException e) {
          break;  //Exit the thread's main loop
        }
      }
    }
  }


  class HoistThread implements Runnable {
    long hoistThreadInterval = 100;//Number of milliseconds between the starts of successive
    //iterations
    long iterationStart;
    HoistThread(){ //Constructor
      hoistThreadState = ThreadState.STATE_ZERO; //A state machine must have an initial state
    }
    public void run() {
      while (!Thread.interrupted()) { //A call to "interrupt()" is what stops this thread
        iterationStart = System.currentTimeMillis();
        switch (hoistThreadState) {
          /*
          * See the text of NxtEncoderOp.java for a simple example of the use of a "switch"
          * statement to implement a state machine. The names of each state and the contents of
          * the corresponding "case" blocks may be whatever the programmers deem necessary.
          */
          case STATE_ZERO: {
              /*
              * In this skeletal example, the code for this state might control how far the robot
              * hoists a delivery mechanism to a particular height. Iteration by iteration, this
              * code can use successive values of (motorHoistCurrentEncoder-motorHoistEncoderOffset)
              * to generate successive values of hoisttMotorPower. When this state's goal has been
              * reached, it can set hoistThreadState to point to a successor state, which might, for
              * example, activate a dumping mechanism.
              */
            break;
          }
          default: {
            break;
          }
        }
        try { //When the work of each iteration is done, "sleep" until time to start the next one.
          //The "sleep" time required depends on the duration of the iteration just ended.
          Thread.sleep(hoistThreadInterval + Math.max(-hoistThreadInterval,
                                                       (iterationStart - System.currentTimeMillis())));
        } catch (InterruptedException e) {
          break;  //Exit the thread's main loop
        }
      }
    }
  }


  /************************************************************************************************
   * The following method was introduced in the 3 August 2015 FTC SDK beta release and it runs
   * before "start" runs.
  */
  @Override
  public void init() {
    //Is this code best executed here, or in the Constructor?
    autoDriver = new Thread(new DriverThread());
    autoHoist = new Thread(new HoistThread());
  }


  /************************************************************************************************
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {
        /*
      	* Use the hardwareMap to get the dc motors, servos and other sensors by name. Note
      	* that the names of the devices must match the names used when you
      	* configured your robot and created the configuration file. The hardware map
      	* for this OpMode is not initialized until the OpModeManager's "startActiveOpMode" method
      	* runs.
    		*/
    motorRight = hardwareMap.dcMotor.get("motor_2");
    motorLeft = hardwareMap.dcMotor.get("motor_1");
    motorLeft.setDirection(DcMotor.Direction.REVERSE);
    motorHoist = hardwareMap.dcMotor.get("motor_3");
    try {
    boschBNO055 = new AdafruitIMU(hardwareMap, "bno055","CDIM_2", 5, (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2),(byte)AdafruitIMU.OPERATION_MODE_IMU);
    } catch (RobotCoreException e){
      Log.i("FtcRobotController", "Exception: " + e.getMessage());
    }
                                //ADDRESS_B is the "standard" I2C bus address for the Bosch BNO055.
                                //??Does this instantiation of an I2cDevice work??
    //Since the DcMotor class currently offers no way to reset DC motor encoders, the initial
    // encoder values must be saved as offsets to be subtracted from future encoder readings
    motorLeftEncoderOffset = motorLeft.getCurrentPosition();
    motorRightEncoderOffset = motorRight.getCurrentPosition();
    motorHoistEncoderOffset = motorHoist.getCurrentPosition();
   // boschBNO055.initIMU(AdafruitIMU.OPERATION_MODE_IMUPLUS);//Set up the IMU as needed for I2C reads
    // and writes. IMUPLUS is an appropriate operational mode for FTC competitions. (See the IMU
    // datasheet.)
    autoDriver.start(); //Get the "worker" threads working!
    autoHoist.start();
  }


  /***********************************************************************************************
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   * NOTE: BECAUSE THIS "loop" METHOD IS PART OF THE OVERALL OpMode/EventLoop/ReadWriteRunnable
   * MECHANISM, ALL THAT THIS METHOD WILL BE USED FOR, IN AUTONOMOUS MODE, IS TO:
   * 1. READ SENSORS AND ENCODERS AND STORE THEIR VALUES IN SHARED VARIABLES
   * 2. WRITE MOTOR POWER AND CONTROL VALUES STORED IN SHARED VARIABLES BY "WORKER" THREADS, AND
   * 3. SEND TELELMETRY DATA TO THE DRIVER STATION
   * THIS "loop" METHOD IS THE ONLY ONE THAT "TOUCHES" ANY SENSOR OR MOTOR HARDWARE.
   */
  @Override
  public void loop() {
    // write the values computed by the "worker" threads to the motors
    motorRight.setPower(Range.clip(rightMotorPower, -1, 1));
    motorLeft.setPower(Range.clip(leftMotorPower, -1, 1));
    motorHoist.setPower(Range.clip(hoistMotorPower, -1, 1));

    //Read the encoder values that the "worker" threads will use in their computations
    motorLeftCurrentEncoder = motorLeft.getCurrentPosition();
    motorRightCurrentEncoder = motorRight.getCurrentPosition();
    motorHoistCurrentEncoder = motorHoist.getCurrentPosition();
    headingAngle = boschBNO055.getIMUHeadingAngle();

		/*
		 * Send whatever telemetry data you want back to driver station.
		 */
    telemetry.addData("Text", "*** Robot Data***");
    telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f",
                                                                     Range.clip(leftMotorPower, -1, 1)));
    telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f",
                                                                      Range.clip(rightMotorPower, -1, 1)));
    telemetry.addData("heading(yaw) angle", "IMU heading: "
                      + String.format("%d, Hex: 0X%4X", headingAngle, headingAngle));
  }


  /*
  * Code to run when the op mode is first disabled goes here
  * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
  */
  @Override
  public void stop() {
    //When the FTC Driver Station's "Start with Timer" button commands autonomous mode to start,
    //then stop after 30 seconds, stop the motors immediately!
    motorRight.setPower(0.0);
    motorLeft.setPower(0.0);
    motorHoist.setPower(0.0);
    autoDriver.interrupt(); //Stop the "worker" threads
    autoHoist.interrupt();
    //Following this method, the underlying FTC system will call a "stop" routine of its own
  }
}
