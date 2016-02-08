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
    private Servo servoCliffHanger = null;
    private long initTimer = System.nanoTime();


    private boolean climberEngaged = false;
    private boolean climberUp = false;
    private int targetPosTicks;
    private double targetPosMeters;
    private long ticksPerMeterCliffHanger = 11687; //actual measured value
    private long ticksPerInchCliffHanger = (long)(ticksPerMeterCliffHanger / 39.3701); //used to verify the tick values vs the tape measure
    int initCase = 0;
    int climPrevPos = 0;
    private final static int climberRelaxed = 2250;
    private final static int climberStartPos = 900;
    private final static int climberButton = 2250;
    public static int servoOffset = 280;
    private final static int cliffEngage = 1850;//1637
    private final static int cliffClear = 1730; //1577
    private final static int mtnEngage = 1850 + servoOffset;
    private final static int mtnClear = 1800 + servoOffset;

    public CliffHanger(DcMotor cliffHanger1, DcMotor cliffHanger2, Servo servoCliffHanger) {
        this.cliffHanger1 = cliffHanger1;
        this.cliffHanger2 = cliffHanger2;
        this.servoCliffHanger = servoCliffHanger;
        setCliffMode(DcMotorController.RunMode.RESET_ENCODERS);
        targetPosMeters = 0;
        targetPosTicks = 0;
    }

    public void cliffInit() {

        switch(initCase) {
            case 0: //thing
                setCliffMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                servoCliffHanger.setPosition(climberRelaxed);
                initCase++;
                break;
            case 1: //begin retracting tape measure
                setCliffPower(-.20);
                initTimer = System.nanoTime();
                initCase++;
                break;
            case 2: //determine if the tape has finished retracting
                if (System.nanoTime() - initTimer > 1e8) {
                    if (cliffHanger1.getCurrentPosition() == climPrevPos) {
                        setCliffPower(0);
                        setCliffMode(DcMotorController.RunMode.RESET_ENCODERS);
                        initCase++;
                    }
                    initTimer = System.nanoTime();
                }
                climPrevPos = cliffHanger1.getCurrentPosition();

                break;
            case 3: //begin extending out to .1m (10cm)
                setCliffMode(DcMotorController.RunMode.RESET_ENCODERS);
                setCliffMode(DcMotorController.RunMode.RUN_TO_POSITION);
                setCliffHangerPos(0.1);
                setCliffPower(.5);
                initTimer = System.nanoTime();
                initCase++;
                break;
            case 4: //checks to see if the tape has finished extending
                if(cliffHanger1.getCurrentPosition() > cliffHanger1.getTargetPosition() - 50) {
                    setCliffMode(DcMotorController.RunMode.RESET_ENCODERS);
                    setCliffMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    setCliffHangerPos(0);
                    setCliffPower(0);
                    initCase++;
                }
                break;
            case 5: //lift the servo so that the robot fits in the sizing cube
                servoCliffHanger.setPosition(climberStartPos);
                initCase++;
                break;
            default:
                setCliffPower(0);
                break;
        }
    }


    public long getTicksPerMeterCliffHanger() { return ticksPerMeterCliffHanger; }

    public long getTicksPerInchClimber() { return ticksPerInchCliffHanger; }

    public boolean getCliffUp() { return climberUp; }

    public boolean getCliffEngaged() { return climberEngaged; }

    public DcMotorController.RunMode getCliffMode() { return cliffHanger1.getMode(); }

    public int getCliffMotorPos() { return cliffHanger1.getCurrentPosition(); }

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
    public void setCliffHangerPos(double metersOut)
    {
        cliffHanger1.setTargetPosition(calcCliffHangerTarget(metersOut));
        cliffHanger2.setTargetPosition(cliffHanger1.getTargetPosition());
    }
    public void setCliffPower(double pwr) {
        cliffHanger1.setPower(pwr);
        cliffHanger2.setPower(pwr);
    }
    public void setCliffMode(DcMotorController.RunMode runMode) {
        cliffHanger1.setMode(runMode);
        cliffHanger2.setMode(runMode);
    }

    public void setCliffServoPos(int pulse) {
        servoCliffHanger.setPosition(ServoNormalize(pulse));
    }
    public void setCliffEngaged(boolean engaged) {climberEngaged = engaged;}

    public void setCliffUp(boolean isUp) {climberUp = isUp;}

    public void updateServoElevation() {
        int pulse = ClimberAngle(climberUp, climberEngaged);
        setCliffServoPos(pulse);

    }

    public void resetCliffInit() {
        initCase = 0;
    }

    public void setCliffServoNonClimbing(String position){
        switch(position) {
            case "Start":
                setCliffServoPos(climberStartPos);
                break;
            case "Relaxed":
                setCliffServoPos(climberRelaxed);
                break;
            case "Button":
                setCliffServoPos(climberButton);
                break;
            default:
                break;
        }
    }


    private int ClimberAngle (boolean up, boolean engage){
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
        return pulse; //convert mr servo controller pulse width to double on 0 - 1 scale
    }

    public double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on 0 - 1 scale
    }
    public String formatClimber() {
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
}
