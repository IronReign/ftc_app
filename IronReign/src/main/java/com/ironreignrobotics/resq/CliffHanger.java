package com.ironreignrobotics.resq;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tycho on 2/7/2016.
 */

//enum cliffServoPos{
//    Relaxed,
//    Start,
//    Button
//}

public class CliffHanger {

    DcMotor cliffHanger1 = null;
    DcMotor cliffHanger2 = null;
    DcMotor cliffElevation = null;
    private Servo servoCliffHanger = null;
    private long initTimer = System.nanoTime();


    public boolean isInitComplete = false;
    private boolean isClimberEngaged = false;
    private boolean isClimberUp = false;
    private boolean wasClimberEngaged = false;
    private boolean wasClimberUp = false;
    private int targetPosTicks;
    private double targetPosMeters;
    private long ticksPerMeterCliffHanger = 11687; //actual measured value
    private long ticksPerInchCliffHanger = (long)(ticksPerMeterCliffHanger / 39.3701); //used to verify the tick values vs the tape measure
    int initCase = 0;
    int climPrevPos = 0;
    private final static double climberDown = 0;
    private final static double climberStartPos = 90;
//    private final static int climberButton = 2250;
//    public static int servoOffset = 280;
    private final static double cliffEngage = 40.0;
    private final static double cliffClear = 47.0;
    private final static double mtnEngage = 20.0;
    public final static double mtnClear = 28.0;
    private double climberTheta = 0;
    private double ticksPerDegree = 8.0;
    private long cliffRelaxDelay = 0;

    public CliffHanger(DcMotor cliffHanger1, DcMotor cliffHanger2, DcMotor cliffElevation, Servo servoCliffHanger) {
        this.cliffHanger1 = cliffHanger1;
        this.cliffHanger2 = cliffHanger2;
        this.cliffElevation = cliffElevation;
        this.servoCliffHanger = servoCliffHanger;
        setCliffPullMode(DcMotorController.RunMode.RESET_ENCODERS);
        targetPosMeters = 0;
        targetPosTicks = 0;
    }

    public void cliffInit() {

        switch(initCase) {
            case 0: //thing
                setCliffPullMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                setCliffElevationMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                cliffElevation.setPower(-.6);
                servoCliffHanger.setPosition(climberDown);
                initTimer = System.nanoTime();
                initCase++;
                break;
            case 1:
                if (System.nanoTime() - initTimer > 3e8) {
                    if (cliffElevation.getCurrentPosition() <= (climPrevPos + 1) && cliffElevation.getCurrentPosition() >=(climPrevPos - 1)) {
                        cliffElevation.setPower(0);
                        setCliffElevationMode(DcMotorController.RunMode.RESET_ENCODERS);
                        initCase++;

                        climPrevPos = 0;
                    }
                    initTimer = System.nanoTime();
                    climPrevPos = cliffElevation.getCurrentPosition();
                }
                break;
            case 2: //begin retracting tape measure
                setCliffPullPower(-.25);
                if(System.nanoTime() - initTimer > 2e8) {
                    initTimer = System.nanoTime();
                    initCase++;
                }
                break;
            case 3: //determine if the tape has finished retracting
                if (System.nanoTime() - initTimer > 1e8) {
                    if (cliffHanger1.getCurrentPosition() <= (climPrevPos + 1) && cliffHanger1.getCurrentPosition() >=(climPrevPos - 1)) {
                        setCliffPullPower(0);
                        setCliffPullMode(DcMotorController.RunMode.RESET_ENCODERS);
                        initCase++;
                    }
                    initTimer = System.nanoTime();
                    climPrevPos = cliffHanger1.getCurrentPosition();
                }


                break;
            case 4: //begin extending out to .1m (10cm)
                setCliffPullMode(DcMotorController.RunMode.RESET_ENCODERS);
                setCliffPullMode(DcMotorController.RunMode.RUN_TO_POSITION);
                setCliffHangerExtension(0.1);
                setCliffPullPower(.5);
                initTimer = System.nanoTime();
                initCase++;
                break;
            case 5: //checks to see if the tape has finished extending
                if(cliffHanger1.getCurrentPosition() > cliffHanger1.getTargetPosition() - 50) {
                    setCliffPullMode(DcMotorController.RunMode.RESET_ENCODERS);
                    setCliffPullMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    setCliffHangerExtension(0);
                    setCliffPullPower(1);
                    setCliffElevationMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    setCliffElevationPower(.5);
                    initCase++;
                }
                break;
            case 6: //lift the servo so that the robot fits in the sizing cube
                setCliffElevation(climberStartPos);
                setCliffElevation();
                initCase++;
                break;
            default:
                setCliffPullPower(1);
                isInitComplete = true;
                break;
        }
    }


    public long getTicksPerMeterCliffHanger() { return ticksPerMeterCliffHanger; }

    public double getClimberTheta() { return climberTheta; }

    public int getCurrentCliffTheta() { return (int)(cliffElevation.getCurrentPosition() / ticksPerDegree); }

    public long getTicksPerInchClimber() { return ticksPerInchCliffHanger; }

    public boolean getCliffUp() { return isClimberUp; }

    public boolean getCliffEngaged() { return isClimberEngaged; }

    public double getCliffTheta() { return climberTheta; }

    public DcMotorController.RunMode getCliffPullMode() { return cliffHanger1.getMode(); }

    public DcMotorController.RunMode getCliffElevationMode() { return cliffElevation.getMode(); }

    public int getCliffExtension() { return cliffHanger1.getCurrentPosition(); }

    public int getCliffElevation() { return cliffElevation.getCurrentPosition(); }

    public void setTicksPerMeterCliffHanger(long ticksPerMeterCliffHanger) {
        this.ticksPerMeterCliffHanger = ticksPerMeterCliffHanger;
        this.ticksPerInchCliffHanger = (long)(ticksPerMeterCliffHanger / 39.3701);
    }

    public double getCliffPosMeters(DcMotor cliffHanger)
    {
        return (double)(cliffHanger.getCurrentPosition())/ticksPerMeterCliffHanger;

    }


    private int calcCliffHangerTarget(double metersOut)
    {
        return (int) (metersOut * ticksPerMeterCliffHanger);
    }
    private int calcCliffTargetInches(double inchesOut)
    {
        return (int) (calcCliffHangerTarget(inchesOut/ 39.3701));
    }
    public void setCliffHangerExtension(double metersOut)
    {
        cliffHanger1.setTargetPosition(calcCliffHangerTarget(metersOut));
        cliffHanger2.setTargetPosition(cliffHanger1.getTargetPosition());
    }
    public void setCliffElevation(double degreesUp) {
        climberTheta = degreesUp;
    }
    public void setCliffElevation() {
        cliffElevation.setTargetPosition((int) (climberTheta * ticksPerDegree));
        if(cliffRelaxDelay < System.nanoTime() && cliffRelaxDelay > 0){
            cliffRelaxDelay = 0;
            cliffElevation.setPower(0);
        }
    }
    public void setCliffPullPower(double pwr) {
        cliffHanger1.setPower(pwr);
        cliffHanger2.setPower(pwr);
    }
    public void setCliffElevationPower(double pwr) {
        cliffElevation.setPower(pwr);
    }
    public void setCliffPullMode(DcMotorController.RunMode runMode) {
        cliffHanger1.setMode(runMode);
        cliffHanger2.setMode(runMode);
    }
    public void setCliffElevationMode(DcMotorController.RunMode runMode) {
        cliffElevation.setMode(runMode);
    }

    //public void setCliffServoPos(int pulse) {
    //    servoCliffHanger.setPosition(ServoNormalize(pulse));
    //}
    public void setCliffEngaged(boolean engaged) {
        if(isClimberEngaged != engaged && engaged == true)
            cliffRelaxDelay = System.nanoTime() + (long)1e9;
        if (!engaged) cliffElevation.setPower(1);
        isClimberEngaged = engaged;
    }


    public void setCliffUp(boolean isUp) {
        wasClimberUp = isClimberUp;
        isClimberUp = isUp;}

    public void updateServoElevation() {
        //int pulse = ClimberAngle(isClimberUp, isClimberEngaged);
        //setCliffServoPos(pulse);

    }
    public void updateBooleanElevation(){
        ClimberAngle(isClimberUp, isClimberEngaged);
    }

    public void resetCliffInit() {
        initCase = 0;
    }
/*
    public void setCliffServoNonClimbing(String position){
        switch(position) {
            case "Start":
                setCliffServoPos(climberStartPos);
                break;
            case "Relaxed":
                setCliffServoPos(climberDown);
                break;
            case "Button":
                setCliffServoPos(climberButton);
                break;
            default:
                break;
        }
    }
*/


    private void ClimberAngle (boolean up, boolean engage){
        int pulse = 0;
        if(up){
            if(engage) {
                climberTheta = cliffEngage;
            }
            else {

                climberTheta = cliffClear;
            }
        }
        else{
            if(engage) {
                climberTheta = mtnEngage;
            }
            else {
                climberTheta = mtnClear;
            }
        }
    }

    public double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
    public String formatClimber() {
        if(isClimberUp)
        {
            if(isClimberEngaged) return "Cliff Engaged";
            return "Cliff Clear";
        }
        else
        {
            if(isClimberEngaged) return "Mtn Engaged";
            return "Mtn Clear";
        }
    }
}
