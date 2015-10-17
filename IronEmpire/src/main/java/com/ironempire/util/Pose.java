package com.ironempire.util;

public class Pose
{
    private double poseX;
    private double poseY;
    private double poseAngle;
    private double poseSpeed;

    public Pose(double x, double y, double angle, double speed)
    {
        poseX     = x;
        poseY     = y;
        poseAngle = angle;
        poseSpeed = speed;
    }

    public Pose(double x, double y, double angle)
    {
        poseX     = x;
        poseY     = y;
        poseAngle = angle;
        poseSpeed = 0;
    }

    public Pose()
    {
        poseX     = 0;
        poseY     = 0;
        poseAngle = 0;
        poseSpeed = 0;
    }

    public double getX()
    {
        return poseX;
    }

    public double getY()
    {
        return poseY;
    }

    public double getAngle()
    {
        return poseAngle;
    }

    public double getSpeed()
    {
        return poseSpeed;
    }
}
