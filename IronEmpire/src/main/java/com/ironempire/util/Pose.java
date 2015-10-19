package com.ironempire.util;

/**
 * The Pose class stores the current <b>position</b>, <b>angle</b>,
 * and <b>speed</b> of the robot.
 *
 * @author Eliot Partridge
 * @version 0.1
 * @since 2015-10-17
 */
public class Pose
{
    private double poseX;
    private double poseY;
    private double poseAngle;
    private double poseSpeed;

    /**
     * Create a Pose instance that stores all four elements:
     * <var>x</var>, <var>y</var>, <var>angle</var>, and <var>speed</var>.
     *
     * @param x     The position relative to the x axis of the robot
     * @param y     The position relative to the y axis of the robot
     * @param angle The angle of the robot
     * @param speed The speed of the robot
     */
    public Pose(double x, double y, double angle, double speed)
    {
        poseX     = x;
        poseY     = y;
        poseAngle = angle;
        poseSpeed = speed;
    }

    /**
     * Creates a Pose instance with 0 speed, to prevent muscle fatigue
     * by excess typing demand on the software team members. This is likely
     * the one to use on init when speed is zero and starting position is known
     *
     * @param x     The position relative to the x axis of the robot
     * @param y     The position relative to the y axis of the robot
     * @param angle The angle of the robot
     */
    public Pose(double x, double y, double angle)
    {
        poseX     = x;
        poseY     = y;
        poseAngle = angle;
        poseSpeed = 0;
    }

    /**
     * Creates a base Pose instance at the origin, (0,0), with 0 speed and 0 angle.
     * Useful for determining the Pose of the robot relative to the origin.
     */
    public Pose()
    {
        poseX     = 0;
        poseY     = 0;
        poseAngle = 0;
        poseSpeed = 0;
    }

    /**
     * Returns the x position of the robot
     *
     * @return The current x position of the robot
     */
    public double getX()
    {
        return poseX;
    }

    /**
     * Returns the y position of the robot
     *
     * @return The current y position of the robot
     */
    public double getY()
    {
        return poseY;
    }

    /**
     * Returns the angle of the robot
     *
     * @return The current angle of the robot
     */
    public double getAngle()
    {
        return poseAngle;
    }

    /**
     * Returns the speed of the robot
     *
     * @return The current speed of the robot
     */
    public double getSpeed()
    {
        return poseSpeed;
    }
}
