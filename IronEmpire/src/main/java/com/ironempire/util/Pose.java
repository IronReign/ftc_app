package com.ironempire.util;

/**
 * The Pose class stores the current real world position/orientation: <b>position</b>, <b>heading</b>,
 * and <b>speed</b> of the robot.
 *
 * This class should be a point of reference for any navigation classes that want to know current
 * orientation and location of the robot.  The update method must be called regularly, it
 * monitors and integrates data from the orientation (IMU) and odometry (motor encoder) sensors.
 * @author Eliot Partridge, Max Virani
 * @version 0.1
 * @since 2015-10-17
 */

public class Pose
{
    private double poseX;
    private double poseY;
    private double poseHeading;
    private double poseSpeed;
    private double posePitch;
    private double poseRoll;
    private long timeStamp; //timestamp of last update

    /**
     * Create a Pose instance that stores all real world position/orientation elements:
     * <var>x</var>, <var>y</var>, <var>heading</var>, and <var>speed</var>.
     *
     * @param x     The position relative to the x axis of the robot
     * @param y     The position relative to the y axis of the robot
     * @param heading The heading of the robot
     * @param speed The speed of the robot
     */
    public Pose(double x, double y, double heading, double speed)
    {
        poseX     = x;
        poseY     = y;
        poseHeading = heading;
        poseSpeed = speed;
        posePitch = 0;
        poseRoll = 0;
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
        poseHeading = angle;
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
        poseHeading = 0;
        poseSpeed = 0;
        posePitch=0;
        poseRoll=0;
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
    public double getHeading()
    {
        return poseHeading;
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
    public double getPitch() {
        return posePitch;
    }

    public double getRoll() {
        return poseRoll;
    }

    /**
     * Update the current location of the robot. This implementation gets heading and orientation
     * from the Bosch BNO055 IMU and assumes a simple differential steer robot with left and right motor
     * encoders.
     *
     *
     * The current naive implementation assumes an unobstructed robot - it cannot account
     * for running into objects and assumes no slippage in the wheel encoders.  Debris
     * on the field and the mountain ramps will cause problems for this implementation. Further
     * work could be done to compare odometry against IMU integrated displacement calculations to
     * detect stalls and slips
     *
     * This method should be called regularly - about every 20 - 30 milliseconds or so.
     */
    public void Update(){
        long currentTime = System.nanoTime();


        this.timeStamp = currentTime;
    }

}
