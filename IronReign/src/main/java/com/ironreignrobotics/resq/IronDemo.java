package com.ironreignrobotics.resq;

import com.qualcomm.robotcore.hardware.*;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.ironempire.util.*;

/**
 * An example of a synchronous opmode that implements a simple drive-a-bot.
 */
@TeleOp(name="6832 Demo (sync)", group="IronReign")
//@Disabled
public class IronDemo extends SynchronousOpMode
    {
    // All hardware variables can only be initialized inside the main() function,
    // not here at their member variable declarations.
    DcMotor motorLeftBack = null;
    DcMotor motorRightBack = null;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorBeater = null;
    PIDController drivePID = new PIDController(0, 0, 0);
    float ctlLeft;
    float ctlRight;

    public String[] State = {"TeleOp", "Auto", "GoStraight", "GoStraightIMU", "SquareDance"};
    public int stateDex = 0;


        // Our sensors, motors, and other devices go here, along with other long term state
        IBNO055IMU              imu;
        ElapsedTime             elapsed    = new ElapsedTime();
        IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();
        Pose pose;



        // Here we have state we use for updating the dashboard. The first of these is important
        // to read only once per update, as its acquisition is expensive. The remainder, though,
        // could probably be read once per item, at only a small loss in display accuracy.
        EulerAngles angles;
        Position position;
        int                     loopCycles;
        int                     i2cCycles;
        double                  ms;

    @Override
    protected void main() throws InterruptedException {
        // Initialize our hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names you assigned during the robot configuration
        // step you did in the FTC Robot Controller app on the phone.
        //this.motorLeftBack = this.hardwareMap.dcMotor.get("motorLeftBack");
        //this.motorRightBack = this.hardwareMap.dcMotor.get("motorRightBack");
        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");
        this.motorBeater = this.hardwareMap.dcMotor.get("motorBeater");

        // Configure the knobs of the hardware according to how you've wired your
        // robot. Here, we assume that there are no encoders connected to the motors,
        // so we inform the motor objects of that fact.
        //this.motorLeftBack.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //this.motorRightBack.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.motorBeater.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        // One of the two motors (here, the left) should be set to reversed direction
        // so that it can take the same power level values as the other motor.
        //this.motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        this.motorRight.setDirection(DcMotor.Direction.REVERSE);

        // We are expecting the IMU to be attached to an I2C port on  a core device interface
        // module and named "imu". Retrieve that raw I2cDevice and then wrap it in an object that
        // semantically understands this particular kind of sensor.
        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

        // Enable reporting of position using the naive integrator
        imu.startAccelerationIntegration(new Position(), new Velocity());

        //Set up robot pose
        pose = new Pose(0,0,0,0);

        //1120 ticks per rotation of Neverest 40 motor
        //1.5 increase in speed from 24 tooth drive sprocket to 16 tooth driven
        //~80 mm diameter to outside of tread on track sprocket * pi = 251 mm = .25 meter travel per motor turn
        //1120 * 4 = 4480
        //this estimated approach needs to be replaced with a calibrated and measured set of ticks per meter

        pose.setTicksPerMeterLeft(4480);
        pose.setTicksPerMeterRight(4480);


        // Configure the dashboard however we want it
        this.configureDashboard();

        // Wait until we've been given the ok to go
        this.waitForStart();

        // Enter a loop processing all the input we receive
        while (this.opModeIsActive()) {
            if (this.updateGamepads()) {
                // There is (likely) new gamepad input available.
                // Do something with that! Here, we just drive.
                this.doManualDrivingControl(this.gamepad1);
            }

            pose.Update(imu.getAngularOrientation(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());

            switch(stateDex)
            {
                case 0:
                    this.motorLeft.setPower(ctlLeft);
                    this.motorRight.setPower(ctlRight);
                    break;
                case 1:
                    break;
                case 2:
                    motorLeft.setPower(-.8);
                    motorRight.setPower(-.8);
                    break;
                case 3:

                    drivePID.setPID(.01, 0, 0);
                    drivePID.setSetpoint(0);
                    drivePID.enable();
                    //drivePID.setInput(pose.diffAngle(drivePID.getSetpoint(), pose.getHeading()));
                    drivePID.setInputRange(0,360);
                    drivePID.setContinuous();
                    drivePID.setInput(pose.getHeading());
                    motorLeft.setPower(-drivePID.performPID());
                    motorRight.setPower(drivePID.performPID());
                    break;
                default:
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    break;

            }


            // Emit telemetry with the freshest possible values
            this.telemetry.update();

            // Let the rest of the system run until there's a stimulus from the robot controller runtime.
            this.idle();
        }
    }

    /**
     * Implement a simple two-motor driving logic using the left and right
     * right joysticks on the indicated game pad.
     */
    void doManualDrivingControl(Gamepad pad) throws InterruptedException {
        // Remember that the gamepad sticks range from -1 to +1, and that the motor
        // power levels range over the same amount
        ctlLeft = pad.left_stick_y;
        ctlRight = pad.right_stick_y;
        if(pad.y){
            this.motorBeater.setPower(1);
        }
        else if(pad.a){
            this.motorBeater.setPower(0);
        }
        if(pad.right_bumper){
            if(stateDex < 4)
                stateDex++;
            else
                stateDex = 0;
        }
        if(pad.left_bumper){
            if(stateDex > 0)
                stateDex--;
            else
                stateDex = 4;
        }

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
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles     = imu.getAngularOrientation();
            position   = imu.getPosition();

            // The rest of this is pretty cheap to acquire, but we may as well do it
            // all while we're gathering the above.
            loopCycles = getLoopCount();
            i2cCycles  = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
            ms         = elapsed.time() * 1000.0;
        }
        });

        this.telemetry.addLine
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
                telemetry.item("loop count: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return loopCycles;
                    }
                }),
                telemetry.item("i2c cycle count: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return i2cCycles;
                    }
                }));

        telemetry.addLine(
                telemetry.item("loop rate: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatRate(ms / loopCycles);
                    }
                }),
                telemetry.item("i2c cycle rate: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatRate(ms / i2cCycles);
                    }
                }));

        telemetry.addLine(
                telemetry.item("status: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeStatus(imu.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeCalibration(imu.read8(IBNO055IMU.REGISTER.CALIB_STAT));
                    }
                }));

        telemetry.addLine(
                telemetry.item("heading: ", new IFunc<Object>() {
                    public Object value() {
                        return formatAngle(pose.getHeading());
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
                telemetry.item("x: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(pose.getX());
                    }
                }),
                telemetry.item("y: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(pose.getY());
                    }
                }),
                telemetry.item("s: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(pose.getSpeed());
                    }
                }));
        telemetry.addLine(
                telemetry.item("motorLeft: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return motorLeft.getCurrentPosition();
                    }
                }),
                telemetry.item("motorRight: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return motorRight.getCurrentPosition();
                    }
                }));

    }

    // Handy functions for formatting data for the dashboard
    String format(double d) {
        return String.format("%.1f", d);
    }

        String formatAngle(double angle)
        {
            return parameters.angleunit==IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
        }
        String formatRadians(double radians)
        {
            return formatDegrees(degreesFromRadians(radians));
        }
        String formatDegrees(double degrees)
        {

            //return String.format("%.1f", normalizeDegrees(degrees));
            return String.format("%.1f", degrees);
        }
        String formatRate(double cyclesPerSecond)
        {
            return String.format("%.2f", cyclesPerSecond);
        }
        String formatPosition(double coordinate)
        {
            String unit = parameters.accelunit== IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                    ? "m" : "??";
            return String.format("%.2f%s", coordinate, unit);
        }

        //----------------------------------------------------------------------------------------------
        // Utility
        //----------------------------------------------------------------------------------------------

        /** Normalize the angle into the range [-180,180) */
        double normalizeDegrees(double degrees)
        {
            while (degrees >= 180.0) degrees -= 360.0;
            while (degrees < -180.0) degrees += 360.0;
            return degrees;
        }
        double degreesFromRadians(double radians)
        {
            return radians * 180.0 / Math.PI;
        }

        /** Turn a system status into something that's reasonable to show in telemetry */
        String decodeStatus(int status)
        {
            switch (status)
            {
                case 0: return "idle";
                case 1: return "syserr";
                case 2: return "periph";
                case 3: return "sysinit";
                case 4: return "selftest";
                case 5: return "fusion";
                case 6: return "running";
            }
            return "unk";
        }

        /** Turn a calibration code into something that is reasonable to show in telemetry */
        String decodeCalibration(int status)
        {
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
