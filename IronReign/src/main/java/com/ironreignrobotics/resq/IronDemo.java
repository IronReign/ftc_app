package com.ironreignrobotics.resq;

import android.widget.SearchView;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.hardware.*;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.ironempire.util.*;


//Modern robotics servo range is 750 us - 2250 us
/**
 * An example of a synchronous opmode that implements a simple drive-a-bot.
 */
@TeleOp(name = "6832 Demo (sync)", group = "IronReign")
//@Disabled
public class IronDemo extends SynchronousOpMode {

    private boolean active = false;
    public int demoMode = 0;
    public int autoStage = 0;
    public boolean isBlueAlliance = true;


    // All hardware variables can only be initialized inside the main() function,
    // not here at their member variable declarations.
    //DcMotor motorLeftBack = null;
    //DcMotor motorRightBack = null;

    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorBeater = null;
    DcMotor cliffHanger1 = null;
    DcMotor cliffHanger2 = null;
//    volatile DcMotor motorChurros = null;
    Servo servoPlow = null;
    Servo servoClimber = null;
    Servo servoTrough = null;
    Servo servoConveyor = null;

//    public Climber climber;

    //public Thread churroClimber;


    PIDController drivePID = new PIDController(0, 0, 0);
    //NOTE: on isRed, 1 is for blue side and -1 is for red side
    //private int isRed = 1;


    private double KpDrive = .025;
    private double KiDrive = 150;
     private double KdDrive = 0.04;
    private double driveIMUBasePower = .5;
    private double motorPower = 0;
    private double climberPower = 0;
    float ctlLeft;
    float ctlRight;
    int direction = 1;
//    double baseSpeed = 0;
//    double baseHeading = 0;
//    public String[] State = {"TeleOp", "Auto", "GoStraight", "GoStraightIMU", "SquareDance"};

    private boolean climberScheme = false;
    private boolean climberEngaged = false;
    private boolean climberUp = false;
    private long autoPushButtonTimerStart = -1;
    public static int servoOffset = 280;
    private final static int cliffEngage = 1850;//1637
    private final static int cliffClear = 1760; //1577
    private final static int mtnEngage = 1850 + servoOffset;
    private final static int mtnClear = 1800 + servoOffset;
    private boolean demoCase = false;
    private int demoAngle = 0;
    private long climPrevPos = 0;
    private long climberZero = 0;
    private final static int climberDefault = 2250;
    private final static int climberStartPos = 1700;
    private final static int climberButton = 2250;
//  private boolean diagnosticsStarted = false;
//    private boolean tapeRetractFinish = false;
    private int diagnosticCase = 0;
//    private boolean diagnosticsFinished = false;
    private long diagnosticTimer = 0;
//    public String climberPos;
    public final static int troughDown = 1900;
    public final static int troughUp =   1688;
    public final static int troughMid =  1855;
    public final static int conveyorLeft  = 1000;
    public final static int conveyorRight = 2000;
    public final static int conveyorStop  = 1500;
    public final static int plowDown = 1648;
    public final static int plowUp = 1000;
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
        this.servoClimber = this.hardwareMap.servo.get("servoCliff");
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
        pose = new Pose(0, 0, 0, 0);

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

        servoClimber.setPosition(cliffClear);
        // Wait until we've been given the ok to go
        this.waitForStart();


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
                        switch(diagnosticCase) {
                            case 0:
                                climPrevPos = cliffHanger1.getCurrentPosition();
                                diagnosticTimer = System.nanoTime();
                                diagnosticCase++;
                                break;
                            case 1:
                                cliffHanger1.setPower(-.20);
                                cliffHanger2.setPower(-.20);
                                diagnosticTimer = System.nanoTime();
                                diagnosticCase++;
                                break;
                            case 2:
                                if (System.nanoTime() - diagnosticTimer > 1e8) {
                                    if (cliffHanger1.getCurrentPosition() == climPrevPos) {
                                        cliffHanger1.setPower(0);
                                        cliffHanger2.setPower(0);
                                        climPrevPos = cliffHanger1.getCurrentPosition();
                                        climberZero = climPrevPos;
                                        pose.setTicksClimberOffset(climberZero);
                                        diagnosticCase++;
                                    }
                                    diagnosticTimer = System.nanoTime();
                                }
                                climPrevPos = cliffHanger1.getCurrentPosition();

                                break;
                            case 3:
                                cliffHanger1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                                cliffHanger2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                                cliffHanger1.setTargetPosition(pose.calcClimberTarget(cliffHanger1,0.1));
                                cliffHanger2.setTargetPosition(cliffHanger1.getTargetPosition());
                                cliffHanger1.setPower(.15);
                                cliffHanger2.setPower(.15);
                                diagnosticTimer = System.nanoTime();
                                diagnosticCase++;
                                break;
                            case 4: //this will run once without effect as written
                                if(gamepad1.left_trigger > .6)
                                {
                                    cliffHanger1.setPower(-.15);
                                    cliffHanger2.setPower(-.15);
                                }
                                else if(gamepad1.right_trigger > .6)
                                {
                                    cliffHanger1.setPower(.15);
                                    cliffHanger2.setPower(.15);
                                }
                                else
                                {
                                    cliffHanger1.setPower(0);
                                    cliffHanger2.setPower(0);
                                }
                                if(gamepad1.a){
                                    cliffHanger1.setPower(0);
                                    cliffHanger2.setPower(0);
                                    diagnosticCase++;
                                }
                                break;
                            case 5:
                                servoClimber.setPosition(1730);
                                diagnosticCase++;
                                break;
                            default:
                                cliffHanger1.setPower(0);
                                cliffHanger2.setPower(0);
                                break;
                        }
                        break;

                    case 1: //autonomous
                        Autonomous();
                        break;
                    case 2: //tele-op driving
                        this.doManualDrivingControl(this.gamepad1);

                        if (ctlLeft * ctlRight < 0) {
                            this.motorLeft.setPower(ctlLeft);
                            this.motorRight.setPower(ctlRight);
                        } else {
                            this.motorLeft.setPower(ctlLeft * direction);
                            this.motorRight.setPower(ctlRight * direction);
                        }
                        this.cliffHanger1.setPower(climberPower);
                        this.cliffHanger2.setPower(climberPower);

                        break;

                    case 3:
                        MoveIMU(KpDrive, 0, KdDrive, 0, 45, drivePID);
                        break;
                    case 4: //Pre-match diagnostics


                        break;
                    case 5:
                        DemoPID();
                        break;
                    default:
                        motorLeft.setPower(0);
                        motorRight.setPower(0);
                        break;

                }
            }
            else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
                motorBeater.setPower(0);
                pose.setOdometer(0);
                cliffHanger1.setPower(0);
                cliffHanger2.setPower(0);
                servoClimber.getController().pwmDisable();
//                motorChurros.setPower(0);
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
//            climber.stroke();
            climberScheme = !climberScheme;

        }
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
            if(pad.right_trigger >.6)
            {
                climberPower = pad.right_trigger;
            }
            else if(pad.left_trigger >.6)
            {
                climberPower = -pad.left_trigger;
            }
            else
            {
                climberPower = 0;
            }
            if(pad.y)
            {
                climberUp = !climberUp;

            }
            if(pad.b)
            {
                climberEngaged = !climberEngaged;
            }
            if(pad.dpad_up)
            {
                cliffHanger1.setPower(.2);
                cliffHanger2.setPower(.2);
                servoClimber.setPosition(climberDefault);
            }

        }
        else {
            if (pad.dpad_down)//1753 micros
            {
                servoPlow.setPosition(ServoNormalize(plowDown));
            }
            if (pad.dpad_up) {
                servoPlow.setPosition(ServoNormalize(plowUp));
            }
//            if (pad.dpad_left) {
//                servoPlow.setPosition(ServoNormalize(1517)); //1517
//            }
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
            if (pad.left_trigger > 0.6)
            {
                KiDrive -= .001;
            }
            if (pad.right_trigger > 0.6)
            {
                KiDrive += .001;
            }*/
            /*(if (pad.left_trigger > 0.5) {
                motorChurros.setTargetPosition(0);
                motorChurros.setPower(-.5);
            }
            if (pad.right_trigger > 0.5) {
                motorChurros.setTargetPosition(-1550);
                motorChurros.setPower(-.5);
            }*/
        }
        servoClimber.setPosition(ClimberAngle(climberUp, climberEngaged));






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
        //this.motorLeftBack.setPower(ctlLeft);
        //this.motorRightBack.setPower(ctlRight);

    }

    void dexSwitch(Gamepad pad) {
        if (pad.start) {
            active = !active;
            if(active) servoClimber.getController().pwmEnable();
            else servoClimber.getController().pwmDisable();
//            diagnosticsStarted = false;
//            tapeRetractFinish = false;
            diagnosticCase = 0;
        } else if (pad.right_bumper) {
            autoStage = 0;        //Add code preventing bumpers changing state if "halt" is true (halt is made true when state
            if (demoMode < 5)    //changes and made false when some other button is pressed)
                demoMode++;
            else
                demoMode = 0;
            active = false;
//            diagnosticsStarted = false;
//            tapeRetractFinish = false;
            diagnosticCase = 0;

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
            diagnosticCase = 0;
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
                        this.telemetry.item("Climber Ticks:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(cliffHanger1.getCurrentPosition());
                            }
                        })
                );
        this.telemetry.addLine
                (
                        this.telemetry.item("state:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(demoMode);
                            }
                        }),
                        this.telemetry.item("a_stage:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(autoStage);
                            }
                        }),
                        this.telemetry.item(" ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatClimber();
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

    String formatRadians(double radians) {
        return formatDegrees(degreesFromRadians(radians));
    }

    String formatDegrees(double degrees) {

        //return String.format("%.1f", normalizeDegrees(degrees));
        return String.format("%.1f", degrees);
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
    String formatClimber() {
        if(climberUp)
        {
            if(climberEngaged) return "Cliff Engaged";
            return "Cliff Clear";
        }
        else
        {
            if(climberEngaged) return "Mtn Engaged";
            return "Mtn Clear";
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    void Autonomous(){
        switch (autoStage) {
            case 0:   //Drive away from the wall to deploy beater bar ; angle = 0
                pose.setPoseHeading(0);
                pose.Update(imu.getAngularOrientation(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
                servoPlow.setPosition(ServoNormalize(plowDown));
                autoStage++;

                break;
            case 1:   //commented out
                autoStage++; //skipping case 1 right now ********************
                //MoveIMU(KpDrive, KiDrive, KdDrive, driveIMUBasePower, 0, drivePID);
                if (pose.getOdometer() > 0.1) {

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    pose.setOdometer(0);
                    autoStage++;
                }
                break;
            case 2:   //Drive to the beacon; angle = 0
                MoveIMU(KpDrive, KiDrive, KdDrive, driveIMUBasePower, 0, drivePID);
                if (pose.getOdometer() > .45) { //TODO: fix distance value
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    pose.setOdometer(0);
                    autoStage++;
                }
                break;

            case 3:   //rough turn to beacon; angle = 0 to 45(blu) or 360-45(red)
                MoveIMU(KpDrive, KiDrive, KdDrive, 0, allianceAngle(45), drivePID);   //
                if (pose.getHeading() >= allianceAngle(45)-2 && pose.getHeading() <= allianceAngle(45)+ 2) { //TODO: this closeness test won't work right around 0 degrees because of the discontinuity
                    motorLeft.setPower(0);
                    pose.setOdometer(0);
                    autoStage++;
                }
                break;

            case 4:   //Precise turn to beacon - color blob assisted; angle = 45(?)
                MoveRobot(KpDrive, KiDrive, KdDrive, 0, ErrorPixToDeg(x), 0, drivePID);
                if((ErrorPixToDeg(x) < 2) || (ErrorPixToDeg(x) > 358)) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    pose.setOdometer(0);
                    autoStage++;
                }
                break;

            case 5:   //Beacon approach - color blob assisted; angle = 45(?)
                MoveRobot(KpDrive, KiDrive, KdDrive, .25, ErrorPixToDeg(x), 0, drivePID);
                if(pose.getOdometer() >= .25) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    pose.setOdometer(0);
                    autoPushButtonTimerStart = System.nanoTime();
                    autoStage++;
                }
                break;

            case 6:   //Push the button
                int curposCliffHangers = cliffHanger1.getCurrentPosition();
                cliffHanger1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                cliffHanger2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                cliffHanger1.setTargetPosition(pose.calcClimberTarget(cliffHanger1,0.3));
                cliffHanger2.setTargetPosition(cliffHanger1.getTargetPosition());
                cliffHanger1.setPower(.5);
                cliffHanger2.setPower(.5);

                //Retract tape again
                // TODO: should test based on position, not time
                if(System.nanoTime() - autoPushButtonTimerStart > 2500000){
                    //return tape to previous position
                    cliffHanger1.setTargetPosition(curposCliffHangers);
                    cliffHanger2.setTargetPosition(curposCliffHangers);
                    autoStage++;
                    autoPushButtonTimerStart = System.nanoTime();
                }

                break;

            case 7:   //move forward and drop climbers
                servoPlow.setPosition(ServoNormalize(plowUp)); //pull up plow
                motorLeft.setPower(.2);
                motorRight.setPower(.2);
                // TODO: should test based on wall sensor or color blob width
                if(System.nanoTime() - autoPushButtonTimerStart > 2500000){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    autoStage++;
                    autoPushButtonTimerStart = System.nanoTime();
                }
                servoTrough.setPosition(ServoNormalize(troughUp)); //pull up trough to dump climbers
                autoStage++;
                break;

            case 8:   //retreat
//                cliffHanger1.setPower(0);
//                cliffHanger2.setPower(0);
                //servoPlow.setPosition(ServoNormalize(plowDown));
                autoStage++;
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
        if (gamepad1.x){
            demoCase = !demoCase;
        }
        if (gamepad1.right_trigger > .6) {
            demoAngle += 5;
            demoAngle = (int)pose.wrapAngle(demoAngle, 0.0);
        }
        if (gamepad1.right_trigger > .6) {
            demoAngle -= 5;
            demoAngle = (int)pose.wrapAngle(demoAngle, 0.0);
        }
        if(demoCase){
            MoveIMU(KpDrive, KiDrive, KdDrive, 0, demoAngle, drivePID);
        } else {
            if (gamepad1.b) {
                MoveArgos(KpDrive, KiDrive, KdDrive, ErrorPixToDeg(x), 0, drivePID);
            } else {
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
//        PID.setInputRange(0, 360);
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
    double ClimberAngle (boolean up, boolean engage){
        int pulse = 0;
        if(up){
            if(engage)
                pulse = cliffEngage;

            else
                pulse = cliffClear;

        }
        else{
            if(engage)
                pulse = mtnEngage;

            else
                pulse = mtnClear;

        }
        return ServoNormalize(pulse); //convert mr servo controller pulse width to double on 0 - 1 scale
    }
    double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on 0 - 1 scale
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


}
