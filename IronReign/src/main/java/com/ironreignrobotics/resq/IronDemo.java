package com.ironreignrobotics.resq;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.hardware.*;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.ironempire.util.*;
import android.content.Context;


//Modern robotics servo range is 750 us - 2250 us
/**
 * An example of a synchronous opmode that implements a simple drive-a-bot.
 */
@TeleOp(name = "6832 Demo (sync)", group = "IronReign")
//@Disabled Q
public class IronDemo extends SynchronousOpMode {

    private boolean active = false;
    public int demoMode = 0;
    public int autoStage = 0;
    public boolean isBlueAlliance = true;


    // All hardware variables can only be initialized inside the main() function,
    // not here at their member variable declarations.


    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorBeater = null;
    DcMotor cliffHanger1 = null;
    DcMotor cliffHanger2 = null;
    DcMotor cliffElevation = null;
    Servo servoPlow = null;
    Servo servoCliffHanger = null;
    Servo servoTrough = null;
    Servo servoConveyor = null;


    PIDController drivePID = new PIDController(0, 0, 0);
    //NOTE: on isRed, 1 is for blue side and -1 is for red side
    //private int isRed = 1;


    private double KpDrive = .025;
    private double KiDrive = 150;
     private double KdDrive = 0.04;
    private double driveIMUBasePower = .5;
    private double motorPower = 0;
   // private double climberTargetM = _0;
    float ctlLeft;
    float ctlRight;
    int direction = 1;
    private long dpadHorizontalTimer = 0;
    private long aTimer = 0;
    private long bTimer = 0;
    private long xTimer = 0;
    private long yTimer = 0;
    static final private long toggleLockout = (long)3e8; // fractional second lockout between all toggle buttons
    private long toggleOKTime = 0; //when should next toggle be allowed
    private long startTimer = 0;
    private boolean isTroughUp = false;
//    double baseSpeed = _0;
//    double baseHeading = _0;
//    public String[] State = {"TeleOp", "Auto", "GoStraight", "GoStraightIMU", "SquareDance"};

    private boolean climberScheme = false;
    private boolean cliffPreset = true;
    private long autoPushButtonTimerStart = -1;
    private boolean demoCase = false;
    private int demoAngle = 0;


//  private boolean diagnosticsStarted = false;
//    private boolean tapeRetractFinish = false;
//    private boolean diagnosticsFinished = false;
//    public String climberPos;
    public final static int troughDown = 2100;
    public final static int troughUp =   900;
    public final static int troughMid =  1980; //last servo
    public final static int troughRelaxed = 1366;
    public final static int conveyorLeft  = 1000;
    public final static int conveyorRight = 2000;
    public final static int conveyorStop  = 1500;
    public final static int plowDown = 1950;
    public final static int plowUp = 850;
//
// Our sensors, motors, and other devices go here, along with other long term state
    IBNO055IMU imu;
    ElapsedTime elapsed = new ElapsedTime();
    IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();
    Pose pose;


    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    EulerAngles angles;
    Position position;
    int loopCycles;
    int i2cCycles;
    double ms;

    int x = FtcRobotControllerActivity.blobx;

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
        this.motorBeater = this.hardwareMap.dcMotor.get("motorBeater");
        this.servoPlow = this.hardwareMap.servo.get("servoCatcher");
        this.cliffElevation = this.hardwareMap.dcMotor.get("motorCliffElevation");
        this.servoCliffHanger = this.hardwareMap.servo.get("servoCliff");
//        this.motorChurros = this.hardwareMap.dcMotor.get("motorChurros");
        this.cliffHanger1 = this.hardwareMap.dcMotor.get("motorCliffHanger1");
        this.cliffHanger2 = this.hardwareMap.dcMotor.get("motorCliffHanger2");
        this.servoTrough = this.hardwareMap.servo.get("servoTrough");
        this.servoConveyor = this.hardwareMap.servo.get("servoConveyor");
        // Configure the knobs of the hardware according to how you've wired your
        // robot. Here, we assume that there are no encoders connected to the motors,
        // so we inform the motor objects of that fact.
        //this.motorLeftBack.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //this.motorRightBack.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorBeater.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.cliffHanger1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.cliffHanger2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.cliffElevation.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);



        this.motorLeft.setDirection(DcMotor.Direction.REVERSE);
        this.cliffHanger2.setDirection(DcMotor.Direction.REVERSE);


        // We are expecting the IMU to be attached to an I2C port on  a core device interface
        // module and named "imu". Retrieve that raw I2cDevice and then wrap it in an object that
        // semantically understands this particular kind of sensor.
        parameters.angleUnit = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

        // Enable reporting of position using the naive integrator
        //imu.startAccelerationIntegration(new Position(), new Velocity());

        //Set up robot pose
        pose = new Pose(0, 0, 0, 0, cliffHanger1, cliffHanger2, cliffElevation, servoCliffHanger);

        //1120 ticks per rotation of Neverest 40 motor
        //1.5 increase in speed from 24 tooth drive sprocket to 16 tooth driven
        //~80 mm diameter to outside of tread on track sprocket * pi = 251 mm = .25 meter travel per motor turn
        //1120 * 4 = 4480
        //this estimated approach needs to be replaced with a calibrated and measured set of ticks per meter

        pose.setTicksPerMeterLeft(2940);
        pose.setTicksPerMeterRight(2940);
        pose.setOdometer(0);


        // Configure the dashboard however we want it
        this.configureDashboard();

//        servoCliffHanger.setPosition(climberStartPos);
        servoTrough.setPosition(pose.ServoNormalize(troughDown));
        servoConveyor.setPosition(pose.ServoNormalize(conveyorStop));
        //cliffHanger1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //cliffHanger2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //pose.resetCliffInit();
        // Wait until we've been given the ok to go
        //while(!pose.isInitComplete)
        //    pose.cliffInit();


        this.waitForStart();

//        pose.setCliffElevationPower(1);

        // Enter a loop processing all the input we receive
        while (this.opModeIsActive()) {
            if (this.updateGamepads()) {
                // There is (likely) new gamepad input available.
                // Do something with that! Here, we just drive.

                this.dexSwitch(this.gamepad1);
            }

            pose.Update(imu.getAngularOrientation(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            if (active) {
//                climber.run();
                switch (demoMode) {
                    case 0:  //pre-match diagnostics
                        pose.cliffInit();
                        break;

                    case 1: //autonomous
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
                        MoveIMU(KpDrive, 0, KdDrive, 0, 45, drivePID);
                        break;
                    case 4: //Extra testing case
                        if (gamepad1.b) {
                            if(cliffHanger1.getMode().equals(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS))
                            {
                                pose.setCliffPullMode(DcMotorController.RunMode.RUN_TO_POSITION);
                            }
                            else
                            {
                                pose.setCliffPullMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
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
//                motorChurros.setCliffPullPower(_0);
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

            if(climberScheme)
                servoPlow.setPosition(pose.ServoNormalize(plowUp));
        }

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
            if(pad.right_trigger >.6)
            {
                //climberTargetM = pad.right_trigger;
                pose.setCliffHangerExtension(pose.getCliffPosMeters(cliffHanger1) + .1); //set target 1cm beyond current position

            }
            else if(pad.left_trigger >.6)
            {
                //climberTargetM = -pad.left_trigger;
                pose.setCliffHangerExtension(pose.getCliffPosMeters(cliffHanger1) - .1); //set target 1cm below current position
            }
            else
            {
                pose.setCliffHangerExtension(pose.getCliffPosMeters(cliffHanger1));
            }
            if(pad.y) //toggle cliff vs. mountain
            {
                //if(System.nanoTime() - yTimer < 1e8 || System.nanoTime() - yTimer > 1e9) {
                if (toggleAllowed()){
                    pose.setCliffUp(!pose.getCliffUp());
                    pose.setCliffElevationPower(1);
                    pose.updateBooleanElevation();
                    cliffPreset = true;
                }
                //yTimer = System.nanoTime();
            }
            if(pad.b) //toggle engaged vs. clear
            {
                //if(System.nanoTime() - bTimer < 1e8 || System.nanoTime() - bTimer > 1e9) {
                if (toggleAllowed()){
                    pose.setCliffEngaged(!pose.getCliffEngaged());
                    pose.updateBooleanElevation();
                    cliffPreset = true;
                }
                //bTimer = System.nanoTime();
            }
            if(pad.dpad_up) {
                if(cliffElevation.getPower() == 0)
                    pose.setCliffElevationPower(1);
                if(System.nanoTime() - dpadHorizontalTimer < 1e8 || System.nanoTime() - dpadHorizontalTimer > 1e9) {
                    pose.setCliffElevation(pose.getCurrentCliffTheta() + 5);
                    cliffPreset = false;
                }
                dpadHorizontalTimer = System.nanoTime();
            }
            else if(pad.dpad_down) {
                if(cliffElevation.getPower() == 0)
                    pose.setCliffElevationPower(1);
                if(System.nanoTime() - dpadHorizontalTimer < 1e8) {
                    pose.setCliffElevation(pose.getCurrentCliffTheta() - 5);
                    cliffPreset = false;
                }
                dpadHorizontalTimer = System.nanoTime();
            }
            else{
                if(!cliffPreset)
                    pose.setCliffElevation(pose.getCurrentCliffTheta());
            }
            if(pad.dpad_left) { //control conveyor if trough is up, otherwise control direction
                if (isTroughUp)
                    servoConveyor.setPosition(pose.ServoNormalize(conveyorLeft));
                else
                    direction = -1;
            }
            else if(pad.dpad_right) { //control conveyor if trough is up, otherwise control direction
                if (isTroughUp)
                    servoConveyor.setPosition(pose.ServoNormalize(conveyorRight));
                else
                    direction = 1;

            }
            else{
                servoConveyor.setPosition(pose.ServoNormalize(conveyorStop));
            }
            if(pad.a) {
                if (toggleAllowed()) {
                    isTroughUp = !isTroughUp;
                    if (isTroughUp) {
                        servoTrough.setPosition(pose.ServoNormalize(troughUp));

                    } else {
                        servoTrough.setPosition(pose.ServoNormalize(troughMid));
                    }
                }
            }
            //ending of cliff hanger logic

        }
        else {
            if(servoTrough.getPosition() != pose.ServoNormalize(troughMid))
                servoTrough.setPosition(pose.ServoNormalize(troughMid));
            if(servoConveyor.getPosition() != pose.ServoNormalize(conveyorStop))
                servoConveyor.setPosition(pose.ServoNormalize(conveyorStop));
            if (pad.dpad_down)
            {
                servoPlow.setPosition(pose.ServoNormalize(plowDown));
            }
            if (pad.dpad_up) {
                servoPlow.setPosition(pose.ServoNormalize(plowUp));
            }
            if (pad.y) {
                motorBeater.setPower(1);
            }
            if (pad.a) {
                motorBeater.setPower(0);
            }
            if (pad.b) {
                motorBeater.setPower(-1);
            }
            /*

            if (pad.left_trigger > _0.6)
            {
                KiDrive -= .001;
            }
            if (pad.right_trigger > _0.6)
            {
                KiDrive += .001;
            }*/
        }
        pose.updateServoElevation();






        // We're going to assume that the deadzone processing has been taken care of for us
        // already by the underlying system (that appears to be the intent). Were that not
        // the case, then we would here process ctlLeft and ctlRight to be exactly zero
        // within the deadzone.

        // Map the power and steering to have more oomph at low values (optional)
        //  ctlLeft = this.xformDrivingPowerLevels(ctlLeft);
        //ctlRight = this.xformDrivingPowerLevels(ctlRight);

        // Dampen power to avoid clipping so we can still effectively steer even
        // under heavy throttle.
        //
        // We want
        //      -1 <= ctlLeft - ctlRight <= 1
        //      -1 <= ctlLeft + ctlRight <= 1
        // i.e
        //      ctlRight -1 <= ctlLeft <=  ctlRight + 1
        //     -ctlRight -1 <= ctlLeft <= -ctlRight + 1
        //ctlLeft = Range.clip(ctlLeft, ctlRight - 1, ctlRight + 1);
        //ctlLeft = Range.clip(ctlLeft, -ctlRight - 1, -ctlRight + 1);

        // Figure out how much power to send to each motor. Be sure
        // not to ask for too much, or the motor will throw an exception.
        //  float powerLeft = Range.clip(ctlLeft - ctlRight, -1f, 1f);
        ///float powerRight = Range.clip(ctlLeft + ctlRight, -1f, 1f);

        // Tell the motors
        //this.motorLeftBack.setCliffPullPower(ctlLeft);
        //this.motorRightBack.setCliffPullPower(ctlRight);

    }

    void dexSwitch(Gamepad pad) {
        if (pad.start) {

            if(System.nanoTime() - startTimer < 1e8 || System.nanoTime() - startTimer > 1e9) {
                active = !active;
                if (active) servoCliffHanger.getController().pwmEnable();
//            else servoCliffHanger.getController().pwmDisable();
//            diagnosticsStarted = false;
//            tapeRetractFinish = false;
                pose.resetCliffInit();
                pose.setCliffPullMode(DcMotorController.RunMode.RUN_TO_POSITION);
                pose.setCliffPullPower(1);
                pose.setCliffElevationPower(1);
            }
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
            pose.resetCliffInit();

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
            pose.resetCliffInit();
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
                i2cCycles = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
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
                        this.telemetry.item("Climber Power:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(cliffHanger1.getPower());
                            }
                        }),
                        this.telemetry.item("CH Target Angle:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(pose.getClimberTheta());
                            }
                        }),
                        this.telemetry.item("CH Current Angle:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(pose.getCurrentCliffTheta());
                            }
                        }),
                        this.telemetry.item(" ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return pose.formatClimber();
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
                }));
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
        switch (autoStage) {
            case 0:   //Get ready to take off with plow down
                pose.setPoseHeading(0);
                pose.Update(imu.getAngularOrientation(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
                servoPlow.setPosition(pose.ServoNormalize(plowDown));
                SwerveUtil.playSound(hardwareMap.appContext, R.raw.a26);
                autoStage++;

                break;
            case 1:   //commented out
//                autoStage++; //skipping case 1 right now ********************
                MoveIMU(KpDrive, KiDrive, KdDrive, driveIMUBasePower, 0, drivePID);
                if (pose.getOdometer() > 0.1) {

                    //pose.updateBooleanElevation();
                    // lower cliffhanger to trigger beater bar slide out
                    pose.setCliffElevationMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    cliffElevation.setPower(-.6);

                    SwerveUtil.playSound(hardwareMap.appContext, R.raw.a02);

                    autoStage++;
                }
                break;
            case 2:   //Drive to the beacon; angle = _0
                MoveIMU(KpDrive, KiDrive, KdDrive, driveIMUBasePower, 0, drivePID);
                if (pose.getOdometer() > 2.5) { //TODO: fix distance value

                    pose.setCliffElevationMode(DcMotorController.RunMode.RESET_ENCODERS);
                    cliffElevation.setPower(0);
                    pose.setCliffElevationMode(DcMotorController.RunMode.RUN_TO_POSITION);

                    pose.setCliffElevation(pose.mtnClear);
                    pose.setCliffElevationPower(.5);
                    pose.setCliffElevation();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    servoTrough.setPosition(pose.ServoNormalize(troughDown)); //retract trough to allow beater assembly to rotate into position
                    pose.setOdometer(0);
//                    pose.setCliffElevation(15);
                    SwerveUtil.playSound(hardwareMap.appContext, R.raw.a03);
                    autoStage++;
                }
                break;

            case 3:   //rough turn to beacon; angle = _0 to 45(blu) or 360-45(red)
                MoveIMU(KpDrive, KiDrive, KdDrive, 0, allianceAngle(45), drivePID);   //
                if (pose.getHeading() >= allianceAngle(45)-2 && pose.getHeading() <= allianceAngle(45)+ 2) { //TODO: this closeness test won't work right around _0 degrees because of the discontinuity
                    motorLeft.setPower(0);
                    pose.setOdometer(0);
                    SwerveUtil.playSound(hardwareMap.appContext, R.raw.a04);
                    double beaconOffset = pose.wrapAngleMinus(pose.getHeading(), ErrorPixToDeg(x));
                    autoStage++;
                }
                break;

            case 4:   //Precise turn to beacon - color blob assisted; angle = 45(?)
                //MoveRobot(KpDrive, KiDrive, KdDrive, 0, ErrorPixToDeg(x), 0, drivePID);
                //if((ErrorPixToDeg(x) < 2) || (ErrorPixToDeg(x) > 358)) {
               /* MoveIMU(KpDrive, KiDrive, KdDrive, 0, pose.wrapAngle(pose.getHeading(), ErrorPixToDeg(x)), drivePID);
                if (pose.getHeading() >= pose.wrapAngle(pose.getHeading(), ErrorPixToDeg(x)) - 2 && pose.getHeading() <= pose.wrapAngle(pose.getHeading(), ErrorPixToDeg(x))+ 2) { //TODO: this closeness test won't work right around _0 degrees because of the discontinuity
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    pose.setOdometer(0);
                    SwerveUtil.playSound(hardwareMap.appContext, R.raw.a05);
                    autoStage++;
                }
                */
                autoStage++;
                break;

            case 5:   //Beacon approach - color blob assisted; angle = 45(?)
                MoveRobot(KpDrive, KiDrive, KdDrive, .25, ErrorPixToDeg(x), 0, drivePID);
                if(pose.getOdometer() >= .5) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    pose.setOdometer(0);
                    autoPushButtonTimerStart = System.nanoTime();
                    SwerveUtil.playSound(hardwareMap.appContext, R.raw.a06);
                    autoStage++;
                }
                break;

            case 6:   //Push the button
//                int curposCliffHangers = cliffHanger1.getCurrentPosition();
                //Retract tape again
                // TODO: should test based on position, not time
//                if(System.nanoTime() - autoPushButtonTimerStart > 2500000){
                    //return tape to previous position
//                    cliffHanger1.setTargetPosition(curposCliffHangers);
//                   cliffHanger2.setTargetPosition(curposCliffHangers);
//                pose.setCliffHangerExtension(0.3);
//                if(!cliffHanger1.isBusy())
//                    SwerveUtil.playSound(hardwareMap.appContext, R.raw.a07);
                    autoStage++;
//                    autoPushButtonTimerStart = System.nanoTime();
 //               }

                break;
            case 7:
//                pose.setCliffHangerExtension(0);
//                if(!cliffHanger1.isBusy())
//                    SwerveUtil.playSound(hardwareMap.appContext, R.raw.a08);
                    autoStage++;


                break;

            case 8:   //move forward and drop climbers
//                servoPlow.setPosition(pose.ServoNormalize(plowUp)); //pull up plow
//                motorLeft.setPower(.2);
//                motorRight.setPower(.2);
//                 TODO: should test based on wall sensor or color blob width
//                if(System.nanoTime() - autoPushButtonTimerStart > 2500000){
//                    motorLeft.setPower(0);
//                    motorRight.setPower(0);
//                    autoStage++;
//                    autoPushButtonTimerStart = System.nanoTime();
//                }
//                servoTrough.setPosition(pose.ServoNormalize(troughUp)); //pull up trough to dump climbers
//                SwerveUtil.playSound(hardwareMap.appContext, R.raw.a09);
                autoStage++;
                break;

            case 9:   //retreat
//                cliffHanger1.setCliffPullPower(_0);
//                cliffHanger2.setCliffPullPower(_0);
                //servoPlow.setPosition(pose.ServoNormalize(plowDown));

                autoStage++;
                pose.setCliffPullMode(DcMotorController.RunMode.RUN_TO_POSITION);
                break;
            default:
                motorLeft.setPower(0);
                motorRight.setPower(0);
                break;
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
            MoveIMU(KpDrive, KiDrive, KdDrive, 0, demoAngle, drivePID);
        } else {
            if (gamepad1.b) { //hold B to also maintain distance
                MoveArgos(KpDrive, KiDrive, KdDrive, ErrorPixToDeg(x), 0, drivePID);
            } else { //otherwise just center on blob
                MoveRobot(KpDrive, KiDrive, KdDrive, 0, ErrorPixToDeg(x), 0, drivePID);
            }
        }
    }
    void MoveRobot(double Kp, double Ki, double Kd, double pwr, double currentAngle, double targetAngle, PIDController PID) {
        PID.setPID(Kp, Ki, Kd);
        PID.setSetpoint(targetAngle);
        PID.enable();
        //drivePID.setInput(pose.diffAngle(drivePID.getSetpoint(), pose.getHeading()));
        PID.setInputRange(0, 360);
        PID.setContinuous();
        //PID.setInput(pose.getHeading());
        PID.setInput(currentAngle);
        double correction = PID.performPID();
        motorLeft.setPower(pwr + correction);
        motorRight.setPower(pwr - correction);
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

            motorPower /= -3.5;
            motorPower /= 100;
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
        servoCliffHanger.getController().pwmDisable();
        pose.setCliffPullPower(0);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorBeater.setPower(0);
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
