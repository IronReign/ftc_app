package com.ironreignrobotics.resq;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

import com.ironempire.util.*;


//Modern robotics servo range is 750 us - 2250 us
/**
 * An example of a synchronous opmode that implements a simple drive-a-bot.
 */
@TeleOp(name = "Pele (sync)", group = "IronReign")
//@Disabled Q
public class Pele extends SynchronousOpMode {

    private boolean active = false;
    public int demoMode = 0;
    public int autoStage = 0;
    public boolean isBlueAlliance = true;


    // All hardware variables can only be initialized inside the main() function,
    // not here at their member variable declarations.


    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorBeater = null;
    DcMotor flingLeft = null;
    DcMotor flingRight = null;
    DcMotor cliffElevation = null;
    Servo paddleLeft = null;
    Servo paddleRight = null;
    Servo beaterServo = null;
    //Servo servoConveyor = null;


    PIDController drivePID = new PIDController(0, 0, 0);
    //NOTE: on isRed, 1 is for blue side and -1 is for red side
    //private int isRed = 1;


    private double KpDrive = .007;
    private double KiDrive = 0.01;
     private double KdDrive = 0;
    private double driveIMUBasePower = .5;
    private double motorPower = 0;


   // private double climberTargetM = _0;
    float ctlLeft;
    float ctlRight;
    int direction = 1;
    public final double loopTimeMin = .0334; //.0334 seconds = 1/30 of a second - camera frame rate
    public double loopTimeLast = 0;
    public double loopTime = 0;
    private long dpadHorizontalTimer = 0;

    private long aTimer = 0;
    private long bTimer = 0;
    private long xTimer = 0;
    private long yTimer = 0;
    static final private long toggleLockout = (long)3e8; // fractional second lockout between all toggle buttons
    private long toggleOKTime = 0; //when should next toggle be allowed
    private long startTimer = 0;
    private boolean isTroughUp = false;
    private double goalAngle;
//    double baseSpeed = _0;
//    double baseHeading = _0;
//    public String[] State = {"TeleOp", "Auto", "GoStraight", "GoStraightIMU", "SquareDance"};

    private boolean climberScheme = false;
    private boolean cliffPreset = true;
    private long autoPushButtonTimerStart = -1;
    private boolean demoCase = false;
    private int demoAngle = 0;
    private boolean run = false;
    private double lw = 0;
    private double wl = 0;
    protected long autoGPTimer = 0;


//  private boolean diagnosticsStarted = false;
//    private boolean tapeRetractFinish = false;
//    private boolean diagnosticsFinished = false;
//    public String climberPos;
    public final static int paddleLeftOut = 2200; //maybe more
    public final static int paddleLeftIn =   900;
    public final static int paddleRightOut =  800;
    public final static int paddleRightIn = 2100;
    public final static int beaterServoOut  = 900; //not calibrated
    public final static int beaterServoIn = 2000; //not calibrated
  //  public final static int conveyorStop  = 1500;
   // public final static int plowDown = 1950;
   // public final static int plowUp = 850;

    public final static double goalX = 0.0;
    public final static double goalY = 3.05;
    public final static double goalDist = 1.0;
//
// Our sensors, motors, and other devices go here, along with other long term state
    I2cDevice i2cDevice;
    IBNO055IMU imu;
    ElapsedTime elapsed = new ElapsedTime();
    IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();
    public PosePele pose;


    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    EulerAngles angles;
    Position position;
    int loopCycles;
    int i2cCycles;
    double ms;
    public CsvLogKeeper logger;

    int x = FtcRobotControllerActivity.blobx;
    int blobW = FtcRobotControllerActivity.blobWidth;
    int blobH = FtcRobotControllerActivity.blobHeight;


    private double ErrorPixToDeg(int blobx){
        int ViewWidth = 800;
        int ScreenCenterPix;
        int ErrorPix;
        double PixPerDegree;
        double errorDegrees;

        ScreenCenterPix = ViewWidth/2;
        ErrorPix = ScreenCenterPix - blobx;
        PixPerDegree = ViewWidth / 75; //FOV
        errorDegrees = ErrorPix/PixPerDegree;
        if (errorDegrees < 0) {
            errorDegrees += 360;
        }
        return errorDegrees;
    }
    protected void main() throws InterruptedException {

        // Initialize our hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names you assigned during the robot configuration
        // step you did in the FTC Robot Controller app on the phone.
        //this.motorLeftBack = this.hardwareMap.dcMotor.get("motorLeftBack");
        //this.motorRightBack = this.hardwareMap.dcMotor.get("motorRightBack");
        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");

        this.beaterServo = this.hardwareMap.servo.get("beaterServo");

        this.paddleLeft = this.hardwareMap.servo.get("paddleLeft");
        this.paddleRight = this.hardwareMap.servo.get("paddleRight");

        this.flingLeft = this.hardwareMap.dcMotor.get("flingLeft");
        this.flingRight = this.hardwareMap.dcMotor.get("flingRight");

        this.motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        this.flingLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.flingRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);




        this.motorRight.setDirection(DcMotor.Direction.REVERSE);
        this.flingRight.setDirection(DcMotor.Direction.REVERSE);

        // We are expecting the IMU to be attached to an I2C port on  a core device interface
        // module and named "imu". Retrieve that raw I2cDevice and then wrap it in an object that
        // semantically understands this particular kind of sensor.
        parameters.angleUnit = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "BNO055";

        i2cDevice = hardwareMap.i2cDevice.get("bno055");
        imu = ClassFactory.createAdaFruitBNO055IMU(i2cDevice, parameters);

        // Enable reporting of position using the naive integrator
        //imu.startAccelerationIntegration(new Position(), new Velocity());

        //Set up robot pose
        pose = new PosePele(0, 0, 0, 0, flingLeft, flingRight);

        //1120 ticks per rotation of Neverest 40 motor
        //1.5 increase in speed from 24 tooth drive sprocket to 16 tooth driven
        //0.1016m (4") diameter * pi = .31918 meter travel per motor turn
        //reciprocal is number of turns to go 1 meter = 3.133
        //1120 * 3.133 = 3509 ticks per meter
        //this estimated approach needs to be replaced with a calibrated and measured set of ticks per meter

        pose.setTicksPerMeterLeft(3509);
        pose.setTicksPerMeterRight(3509);
        pose.setOdometer(0);

        // Configure the dashboard however we want it
        this.configureDashboard();

        this.waitForStart(); //wait until remote start on driver station

        // Enter a loop processing all the input we receive
        while (this.opModeIsActive()) {
            if (this.updateGamepads()) {
                // There is (likely) new gamepad input available.
                // Do something with that! Here, we just drive.

                this.dexSwitch(this.gamepad1);
            }

            pose.Update(imu.getAngularOrientation(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            x = FtcRobotControllerActivity.blobx;
            blobW = FtcRobotControllerActivity.blobWidth;
            blobH = FtcRobotControllerActivity.blobHeight;
            if (active) {
//                climber.run();
                switch (demoMode) {
                    case 0:  //pre-match diagnostics
                        if (pose.flingerWiggle()) demoMode = 2;


                        break;

                    case 1: //autonomous
                        logger = new CsvLogKeeper("test",9,"NanoTime, Deltatime,Odometer,Error,TotalError,PowerP,PowerI,PowerD,Correction");

                        Autonomous();
                        break;
                    case 2: //tele-op driving
                        this.doManualDrivingControl(this.gamepad1);

                        if (ctlLeft * ctlRight < 0) { //controls are opposing - in place turns should be unaffected by primary direction
                            this.motorRight.setPower(ctlRight);
                            this.motorLeft.setPower(ctlLeft);
                        } else {
                            this.motorLeft.setPower(ctlLeft * direction);
                            this.motorRight.setPower(ctlRight * direction);
                        }


                        break;

                    case 3: //turn to 45 degrees and maintain
                        logger = new CsvLogKeeper("test",9,"NanoTime, Deltatime,Odometer,Error,TotalError,PowerP,PowerI,PowerD,Correction");
                        MoveIMU(KpDrive, 0, KdDrive, 0, 45, drivePID);
                        break;
                    case 4: //Extra testing case
                        if (gamepad1.b) {
                            if(flingLeft.getMode().equals(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS))
                            {

                            }
                            else
                            {

                            }
                        }

                        break;
                    case 5:
                        DemoPID();
                        break;
                    default:
                        relax();
                        break;

                }
            }
            else {
                relax();

            }


            // Emit telemetry with the freshest possible values
            this.telemetry.update();

            // Let the rest of the system run until there's a stimulus from the robot controller runtime.
            this.idle();
        }
    }

    /**
     * +
     * Implement a simple two-motor driving logic using the left and right
     * right joysticks on the indicated game pad.
     */
    void doManualDrivingControl(Gamepad pad) throws InterruptedException {
        // Remember that the gamepad sticks range from -1 to +1, and that the motor
        // power levels range over the same amount
        ctlLeft = -pad.left_stick_y;
        ctlRight = -pad.right_stick_y;

        if (pad.x) {

            if (toggleAllowed()){
                climberScheme = !climberScheme;
            }

            //if(climberScheme)
            //    servoPlow.setPosition(pose.ServoNormalize(plowUp));
        }
/*
        if(pose.getPitch() < 290 && pose.getPitch() > 30) //when near vertical we must be cliffhanging - set trough to relaxed position
            if(servoTrough.getPosition() != pose.ServoNormalize(troughRelaxed))
                servoTrough.setPosition(pose.ServoNormalize(troughRelaxed));

        if(climberScheme)
        {
            if(ctlRight > .6)
            {
                ctlRight = (float).6;
            }
            if(ctlLeft > .6)
            {
                ctlLeft = (float).6;
            }
            if(ctlRight < -.6)
            {
                ctlRight = (float)-.6;
            }
            if(ctlLeft < -.6)
            {
                ctlLeft = (float)-.6;
            }
            //beginning of cliff hanger logic

            */
            if(pad.right_trigger >.6)
            {


            }
            else if(pad.left_trigger >.6)
            {

            }
            else
            {

            }
            if(pad.y)
            {
                pose.fling();
            }
            if(pad.b)
            {
                //paddleLeft.setPosition(pose.ServoNormalize(paddleLeftOut));
                //paddleRight.setPosition(pose.ServoNormalize(paddleRightOut));
                pose.flingRelax();

            }
         if(pad.dpad_up) {
             beaterServo.setPosition(pose.ServoNormalize(beaterServoOut));
            }
            if(pad.dpad_down) {
                beaterServo.setPosition(pose.ServoNormalize(beaterServoIn));

            }
            if(pad.dpad_left) {
                paddleLeft.setPosition(pose.ServoNormalize(paddleLeftOut));
                paddleRight.setPosition(pose.ServoNormalize(paddleRightOut));
            }
            if(pad.dpad_right) { //control conveyor if trough is up, otherwise control direction
                paddleLeft.setPosition(pose.ServoNormalize(paddleLeftIn));
                paddleRight.setPosition(pose.ServoNormalize(paddleRightIn));

            }

            if(pad.a) {
                paddleLeft.setPosition(pose.ServoNormalize(paddleLeftIn));
                paddleRight.setPosition(pose.ServoNormalize(paddleRightIn));
                }
            }


    void dexSwitch(Gamepad pad) {
        if (pad.start) {
            active=!active;

            startTimer = System.nanoTime();
        } else if (pad.right_bumper) {
            autoStage = 0;        //Add code preventing bumpers changing state if "halt" is true (halt is made true when state
            if (demoMode < 5)    //changes and made false when some other button is pressed)
                demoMode++;
            else
                demoMode = 0;
            active = false;
//            diagnosticsStarted = false;
//            tapeRetractFinish = false;


        } else if (pad.left_bumper) {
            autoStage = 0;
            if (demoMode > 0)
                demoMode--;
            else
                demoMode = 5;
            active = false;
//            diagnosticsStarted = false;
//            tapeRetractFinish = false;
//            diagnosticsFinished = false;

        }

    }

    float xformDrivingPowerLevels(float level)
    // A useful thing to do in some robots is to map the power levels so that
    // low power levels have more power than they otherwise would. This sometimes
    // help give better driveability.
    {
        // We use a log function here as a simple way to transform the levels.
        // You might want to try something different: perhaps construct a
        // manually specified function using a table of values over which
        // you interpolate.
        float zeroToOne = Math.abs(level);
        float oneToTen = zeroToOne * 9 + 1;
        return (float) (Math.log10(oneToTen) * Math.signum(level));
    }

    void configureDashboard() {
        // Configure the dashboard.

        // The default dashboard update rate is a little too slow for our taste here, so we update faster
        telemetry.setUpdateIntervalMs(200);

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation();
                position = imu.getPosition();

                // The rest of this is pretty cheap to acquire, but we may as well do it
                // all while we're gathering the above.
                loopCycles = getLoopCount();
                //i2cCycles = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
                i2cCycles  = i2cDevice.getCallbackCount();
                ms = elapsed.time() * 1000.0;
                x = FtcRobotControllerActivity.blobx;
            }
        });
        this.telemetry.addLine
                (
                        this.telemetry.item("state:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(demoMode);
                            }
                        }),
                        this.telemetry.item("active:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(active);
                            }
                        }),
                        this.telemetry.item("", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatMountainMode();
                            }
                        })
                        );
        this.telemetry.addLine
                (
                        this.telemetry.item("Flinger Power:", new IFunc<Object>() {
                                    @Override
                                    public Object value() {
                                        return format(flingLeft.getPower());
                                    }
                                })
                );
        this.telemetry.addLine
                (
                        this.telemetry.item("Kp:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatPosition(KpDrive);
                            }
                        }),
                        this.telemetry.item("Ki:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatPosition(KiDrive);
                            }
                        }),
                        this.telemetry.item("Kd:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatPosition(KdDrive);
                            }
                        }),
                        this.telemetry.item("Total Error:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatPosition(drivePID.m_totalError);
                            }
                        })

                );
        this.telemetry.addLine
                (
                        this.telemetry.item("Kp Result:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatSmallNum(drivePID.pwrP);
                            }
                        }),
                        this.telemetry.item("Ki Result:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatSmallNum(drivePID.pwrI);
                            }
                        }),
                        this.telemetry.item("Kd Result:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatSmallNum(drivePID.pwrD);
                            }
                        }),
                        this.telemetry.item("Delta T:", new IFunc<Object>() {
                            @Override
                             public Object value() {
                                return formatSmallNum(drivePID.m_deltaTime);
                             }
                        })
                );

        this.telemetry.addLine //color blob detector stuff
                (

                        this.telemetry.item("BlobErr:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(ErrorPixToDeg(x));
                            }
                        }),
                        this.telemetry.item("BlobSize:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(Math.sqrt(FtcRobotControllerActivity.maxContour));
                            }
                        }),
                        this.telemetry.item("Hue:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(FtcRobotControllerActivity.mBlobColorHsv.val[0]);
                            }
                        })

                );
        telemetry.addLine
                (
                        this.telemetry.item("left:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(motorLeft.getPower());
                            }
                        }),
                        this.telemetry.item("right: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(motorRight.getPower());
                            }
                        }),
                        this.telemetry.item("mode: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return motorLeft.getMode();
                            }
                        })
                );
        telemetry.addLine(
                telemetry.item("loop count: ", new IFunc<Object>() {
                    public Object value() {
                        return loopCycles;
                    }
                }),
                telemetry.item("i2c cycle count: ", new IFunc<Object>() {
                    public Object value() {
                        return i2cCycles;
                    }
                }),
                telemetry.item("ArgosPower:", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return formatPosition(motorPower);
                    }
                })
        );

        telemetry.addLine(
                telemetry.item("loop rate: ", new IFunc<Object>() {
                    public Object value() {
                        return formatRate(ms / loopCycles);
                    }
                }),
                telemetry.item("i2c cycle rate: ", new IFunc<Object>() {
                    public Object value() {
                        return formatRate(ms / i2cCycles);
                    }
                }));

        telemetry.addLine(
                telemetry.item("status: ", new IFunc<Object>() {
                    public Object value() {
                        return decodeStatus(imu.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>() {
                    public Object value() {
                        return decodeCalibration(imu.read8(IBNO055IMU.REGISTER.CALIB_STAT));
                    }
                }));

        telemetry.addLine(
                telemetry.item("heading: ", new IFunc<Object>() {
                    public Object value() {
                        return formatAngle(pose.getHeading());
                    }
                }),
                telemetry.item("imu: ", new IFunc<Object>() {
                    public Object value() {
                        return formatAngle(imu.getAngularOrientation().heading);
                    }
                }),
                telemetry.item("roll: ", new IFunc<Object>() {
                    public Object value() {
                        return formatAngle(pose.getRoll());
                    }
                }),
                telemetry.item("pitch: ", new IFunc<Object>() {
                    public Object value() {
                        return formatAngle(pose.getPitch());
                    }
                }));

        telemetry.addLine(
                telemetry.item("x: ", new IFunc<Object>() {
                    public Object value() {
                        return formatPosition(pose.getX());
                    }
                }),
                telemetry.item("y: ", new IFunc<Object>() {
                    public Object value() {
                        return formatPosition(pose.getY());
                    }
                }),
                telemetry.item("odo: ", new IFunc<Object>() {
                    public Object value() {
                        return formatPosition(pose.getOdometer());
                    }
                }));
        telemetry.addLine(
                telemetry.item("motorLeft: ", new IFunc<Object>() {
                    public Object value() {
                        return motorLeft.getCurrentPosition();
                    }
                }),
                telemetry.item("motorRight: ", new IFunc<Object>() {
                    public Object value() {
                        return motorRight.getCurrentPosition();
                    }
                }));
        telemetry.addLine(
                telemetry.item("blobx: ", new IFunc<Object>() {
                    public Object value() {
                        return x;
                    }
                }),
                telemetry.item("blobW: ", new IFunc<Object>() {
                    public Object value() {
                        return blobW;
                    }
                }),
                telemetry.item("blobH: ", new IFunc<Object>() {
                    public Object value() {
                        return blobH;
                    }
                })
        );
    }


    // Handy functions for formatting data for the dashboard
    String format(double d) {
        return String.format("%.1f", d);
    }

    String formatAngle(double angle) {
        return parameters.angleUnit == IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
    }

    String format(boolean b) { return b + ""; }

    String formatRadians(double radians) {
        return formatDegrees(degreesFromRadians(radians));
    }

    String formatDegrees(double degrees) {

        //return String.format("%.1f", normalizeDegrees(degrees));
        return String.format("%.1f", degrees);
    }

    String formatMotorMode(DcMotorController.RunMode s){
        return s.toString();
    }

    String formatMountainMode() {
        if(climberScheme)
            return "Mountain";
        return "Floor";
    }

    String formatRate(double cyclesPerSecond) {
        return String.format("%.2f", cyclesPerSecond);
    }
    String formatSmallNum(double cyclesPerSecond) {
        return String.format("%.6f", cyclesPerSecond);
    }

    String formatPosition(double coordinate) {
        String unit = parameters.accelUnit == IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m" : "??";
        return String.format("%.3f%s", coordinate, unit);
    }


    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    void Autonomous(){
        loopTime = System.nanoTime();
        if (loopTime-loopTimeLast>loopTimeMin) { //skip over this loop if racing through
            loopTimeLast = loopTime;
            if(run){
                run = false;
            }
            else {
                switch (autoStage) {
                    case 0:
                        paddleLeft.setPosition(pose.ServoNormalize(paddleLeftOut));
                        paddleRight.setPosition(pose.ServoNormalize(paddleRightOut));

                        motorLeft.setPower(.07);
                        motorRight.setPower(-.07 );
                        autoStage++;
                        break;
                    case 1:
                        if (blobH>40){  //a can is probably in view - stop motors and set up the drive to it
                            drivePID.reset();
                            motorLeft.setPower(0);
                            motorRight.setPower(0);
                            autoStage++; //found a blob large enough to pursue
                        }
                        break;
                    case 2: //approach can

                        MoveCan(KpDrive, KiDrive, KdDrive, ErrorPixToDeg(x), 0, drivePID);
                        if (blobH>150 && blobW > 130) {  //an upright can is close enough to grab
                            motorLeft.setPower(0); //stop
                            motorRight.setPower(0);

                            //topple can into flinger
                            beaterServo.setPosition(pose.ServoNormalize(beaterServoIn));

                            autoGPTimer=(long)loopTime + (long)1e9;
                            autoStage++;
                        }
                        if (blobH>25 && blobW > 250) {  //a toppled can is close enough to grab

                            autoStage=4;
                        }

                        break;

                    case 3:
                        if (autoGPTimer<(long)loopTime) //wait for servo to tip can in
                        {
                            beaterServo.setPosition(pose.ServoNormalize(beaterServoOut));
                            autoGPTimer=(long)loopTime + (long)1e9;
                            autoStage++;
                        }
                        break;

                    case 4:
                    if (autoGPTimer<(long)loopTime) //wait for tipping paddle to get out of the way
                    {
                        paddleLeft.setPosition(pose.ServoNormalize(paddleLeftIn));
                        paddleRight.setPosition(pose.ServoNormalize(paddleRightIn));
                        autoGPTimer=(long)loopTime + (long).5e9; //time to let gates close
                        autoStage++;
                    }
                    break;

                    case 5: //wait for gates to close
                        if (autoGPTimer<(long)loopTime) autoStage++;
                        break;

                    case 6: //wiggle

                        if (pose.flingerWiggle()) {
                            drivePID.reset();
                            goalAngle = pose.getBearingOpposite(goalX, goalY);
                            autoStage++;
                            autoGPTimer=(long)loopTime + (long)5e9; //set turn-around max duration
                        }

                        break;

                    case 7: //turn toward goal
                        MoveIMU(KpDrive, 0, KdDrive, 0, goalAngle, drivePID);
                        if (autoGPTimer<(long)loopTime) //wait for turnabout - this is a very simple method - should wait until we settle on a close-enough target heading?
                        {
                            motorLeft.setPower(0); //stop
                            motorRight.setPower(0);

                            autoStage++;

                        }
                        break;

                    case 8: //shoot

                        if (pose.fling()) autoStage++; //fling until flung
                        break;

                    case 9: //relax

                        if (pose.flingRelax()) autoStage = 0; //wait until relaxed then loop back to look for another can
                                //todo: add logic to terminate when no more cans are found?
                        break;

                    //Upright can ready to be grabbed: blobH around 217, blowW around 157 but more variable due to specular changes
                    //Horizontal can ready to be grabbed:  blobW near 280 to 295, blobH variable 95-150 and touching bottom of screen
                    //Can is in flinger when blobW near 360 +-20 and blowH near 75 +-15, touching bottom of screen


                    default:
                        motorLeft.setPower(0);
                        motorRight.setPower(0);
                        break;
                }
            }

        }

    }
    void DemoPID(){
        if (gamepad1.dpad_left)
        {
            KpDrive -= .001;

        }
        if (gamepad1.dpad_right) {
            KpDrive += .001;
        }
        if (gamepad1.a)
        {
            KiDrive -= .001;
        }
        if (gamepad1.y) {
            KiDrive += .001;
        }
        if (gamepad1.dpad_down)
        {
            KdDrive -= .001;
        }
        if (gamepad1.dpad_up) {
            KdDrive += .001;
        }
        if (gamepad1.x){ //toggle between color tracking and imu inputs
            if (toggleAllowed()) {
                demoCase = !demoCase;
            }
        }
        if (gamepad1.right_trigger > .6) {
            demoAngle += 5;
            demoAngle = (int)pose.wrapAngle(demoAngle, 0.0);
        }
        if (gamepad1.right_trigger > .6) {
            demoAngle -= 5;
            demoAngle = (int)pose.wrapAngle(demoAngle, 0.0);
        }

        if(demoCase){ //use X to toggle between color tracking and IMU inputs
            MoveCan(KpDrive, KiDrive, KdDrive, 0, demoAngle, drivePID);
        } else {
            if (gamepad1.b) { //hold B to also maintain distance
                MoveArgos(KpDrive, KiDrive, KdDrive, ErrorPixToDeg(x), 0, drivePID);
            } else { //otherwise just center on blob
                MoveRobot(KpDrive, KiDrive, KdDrive, 0, ErrorPixToDeg(x), 0, drivePID);
            }
        }
    }
    void MoveRobot(double Kp, double Ki, double Kd, double pwr, double currentAngle, double targetAngle, PIDController PID) {
        //if (pwr>0) PID.setOutputRange(pwr-(1-pwr),1-pwr);
        //else PID.setOutputRange(pwr - (-1 - pwr),-1-pwr);
        PID.setOutputRange(-.5,.5);
        PID.setPID(Kp, Ki, Kd);
        PID.setSetpoint(targetAngle);
        PID.enable();

        PID.setInputRange(0, 360);
        PID.setContinuous();
        PID.setInput(currentAngle);
        double correction = PID.performPID();
        /*ArrayList toUpdate = new ArrayList();
        toUpdate.add((PID.m_deltaTime));
        toUpdate.add(Double.valueOf(PID.m_error));
        toUpdate.add(new Double(PID.m_totalError));
        toUpdate.add(new Double(PID.pwrP));
        toUpdate.add(new Double(PID.pwrI));
        toUpdate.add(new Double(PID.pwrD));*/
/*
        logger.UpdateLog(Long.toString(System.nanoTime()) + ","
                + Double.toString(PID.m_deltaTime) + ","
                + Double.toString(pose.getOdometer()) + ","
                + Double.toString(PID.m_error) + ","
                + Double.toString(PID.m_totalError) + ","
                + Double.toString(PID.pwrP) + ","
                + Double.toString(PID.pwrI) + ","
                + Double.toString(PID.pwrD) + ","
                + Double.toString(correction));
        motorLeft.setPower(pwr + correction);
        motorRight.setPower(pwr - correction
*/
    }

    void MoveArgos(double Kp, double Ki, double Kd, double currentAngle, double targetAngle, PIDController PID){

//        PID.setPID(Kp, Ki, Kd);
//        PID.setSetpoint(targetAngle);
//        PID.enable();
//        //drivePID.setInput(pose.diffAngle(drivePID.getSetpoint(), pose.getHeading()));
//        PID.setInputRange(_0, 360);
//        PID.setContinuous();
//        //PID.setInput(pose.getHeading());
//        PID.setInput(currentAngle);

        if((FtcRobotControllerActivity.maxContour >0) && (FtcRobotControllerActivity.targetContour > 0))
        {
            motorPower = Math.sqrt(FtcRobotControllerActivity.targetContour) - Math.sqrt(FtcRobotControllerActivity.maxContour);

            motorPower /= 3.5;
            motorPower /= 100;
        }
//        if(motorPower > .5)
//            motorPower = .5;
//        if(motorPower < -.5)
//            motorPower = -.5;
        MoveRobot(KpDrive, 0, KdDrive, motorPower, ErrorPixToDeg(x), 0, drivePID);

    }
    void MoveCan(double Kp, double Ki, double Kd, double currentAngle, double targetAngle, PIDController PID){

//        PID.setPID(Kp, Ki, Kd);
//        PID.setSetpoint(targetAngle);
//        PID.enable();
//        //drivePID.setInput(pose.diffAngle(drivePID.getSetpoint(), pose.getHeading()));
//        PID.setInputRange(_0, 360);
//        PID.setContinuous();
//        //PID.setInput(pose.getHeading());
//        PID.setInput(currentAngle);

        if((FtcRobotControllerActivity.maxContour >0) && (FtcRobotControllerActivity.targetContour > 0))
        {
            motorPower = .15;
        }
//        if(motorPower > .5)
//            motorPower = .5;
//        if(motorPower < -.5)
//            motorPower = -.5;
        MoveRobot(KpDrive, 0, KdDrive, motorPower, ErrorPixToDeg(x), 0, drivePID);

    }

    void MoveIMU(double Kp, double Ki, double Kd, double pwr, double angle, PIDController PID) {
        MoveRobot(Kp, Ki, Kd, pwr, pose.getHeading(), angle, drivePID);
    }
double allianceAngle(double blueAngle) {
    if (isBlueAlliance)
        return blueAngle;
    else
        return 360 - blueAngle;

}


    double normalizeDegrees(double degrees) {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    double degreesFromRadians(double radians) {
        return radians * 180.0 / Math.PI;
    }

    /**
     * Turn a system status into something that's reasonable to show in telemetry
     */
    String decodeStatus(int status) {
        switch (status) {
            case 0:
                return "idle";
            case 1:
                return "syserr";
            case 2:
                return "periph";
            case 3:
                return "sysinit";
            case 4:
                return "selftest";
            case 5:
                return "fusion";
            case 6:
                return "running";
        }
        return "unk";
    }

    /**
     * Turn a calibration code into something that is reasonable to show in telemetry
     */
    String decodeCalibration(int status) {
        StringBuilder result = new StringBuilder();

        result.append(String.format("s%d", (status >> 2) & 0x03));  // SYS calibration status
        result.append(" ");
        result.append(String.format("g%d", (status >> 2) & 0x03));  // GYR calibration status
        result.append(" ");
        result.append(String.format("a%d", (status >> 2) & 0x03));  // ACC calibration status
        result.append(" ");
        result.append(String.format("m%d", (status >> 0) & 0x03));  // MAG calibration status

        return result.toString();
    }
    void relax() {


        motorLeft.setPower(0);
        motorRight.setPower(0);

        pose.setOdometer(0);
    }

boolean toggleAllowed()
{
    if (System.nanoTime()> toggleOKTime)
    {
        toggleOKTime= System.nanoTime()+toggleLockout;
        return true;
    }
    else
        return false;
}


}
