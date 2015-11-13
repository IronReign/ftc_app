package com.ironempire.util;

/**
 * Created by tycho on 11/7/2015.
 */
public class Pilot
{
    private double targetHeading;
    private double targetSpeed;
    private double targetDisplacement;
    private PIDController drivePID;
    private Pose locate;

    /**
     *
     * @param pose
     */
    public Pilot(Pose pose)
    {
        targetHeading = 0;
        targetSpeed = 0;
        targetDisplacement = 0;
        drivePID = new PIDController(0, 0, 0);
        locate = pose;
    }


    /**
     *
     * @param targetHeading (degrees)
     * @param targetSpeed (m/s)
     * @param targetDisplacement (m)
     * @param kP (Proportional scaling factor)
     * @param kI (Integral scaling factor)
     * @param kD (Derivative scaling factor)
     * @param pose (Pose class value)
     */
    public Pilot(double targetHeading, double targetSpeed, double targetDisplacement, double kP, double kI, double kD, Pose pose)
    {
        this.targetHeading = targetHeading;
        this.targetSpeed = targetSpeed;
        this.targetDisplacement = targetDisplacement;
        drivePID = new PIDController(kP, kI, kD);
        locate = pose;
    }

    /**
     *
     * @param targetHeading
     */
    public void setHeading(double targetHeading)
    {
        this.targetHeading =  targetHeading;
    }

    /**
     *
     * @param targetSpeed
     */
    public void setSpeed(double targetSpeed)
    {
        this.targetSpeed =  targetSpeed;
    }

    /**
     *
     * @param targetDisplacement
     */
    public void setDisplacement(double targetDisplacement)
    {
        this.targetDisplacement =  targetDisplacement;
    }

    /**
     *
     * @param kP
     * @param kI
     * @param kD
     */
    public void setPID(double kP, double kI, double kD)
    {
        drivePID.setPID(kP, kI, kD);
    }

    public void goRelative()
    {

        long ticks = 0;
        float powerLeft = 0;
        float powerRight = 0;
        while(ticks <= locate.getTicksPerMeterLeft() * targetDisplacement) {


            powerLeft = (float) drivePID.performPID();
            powerRight = 0 - (float) drivePID.performPID();
            ticks = locate.getTicksLeftPrev();
        }
    }



}
