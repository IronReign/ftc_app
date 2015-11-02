package com.ironempire.util;
import java.util.Scanner;

/**
 * The Theta class stores the current <b>angle</b>,
 * of the robot, and provides some utility functions
 *
 * @author Dylan Chorley
 * @version 0.1
 * @since 2015-10-17
 */


//input is -180 to +180
//make float
public class Theta {
    private float firstNum;
    private boolean conversion;

    /**
     * Theta program is our skeleton program that will be used for doing simple common manipulations
     * on the angle that is recieved from the IMU
     * @param angle= angle is equal to the angle that is recieved by IMU. The angle is in degrees
     * @param convert=convert is equal to if you want to convert the angle from degrees to radians
     */
    public Theta(float angle, boolean convert) {
        firstNum = angle;
        conversion = convert;
    }

    /**
     *This is used so that if our angle is too large or too small can be manipulated so that
     * it fits within the range of -180 to 180 degrees or can change in to radians form from degrees.
     * @param num is equal to the degree of the angle
     * @param convert is equal to if you want to convert the angle or not
     * @return converted float number or the beginning float number in radians or degrees
     */
    public float converter(float num, boolean convert) {
        conversion=convert;
        if (conversion == true) {
            num = (float) ((num * Math.PI) / 180);
            firstNum = num;
            return num;
        }
        if ((conversion == false) && (num <= 180 && num >= -180)) {
            float deg = num;
            firstNum = deg;
            return deg;
        } else {
            if (conversion == false) {

                if (num < -180) {
                    int amount = (int) (num / 360);
                    amount = amount * -1;
                    float deg4 = num + (360 * amount);
                    firstNum = deg4;
                    return deg4;
                } else {
                    if (num > 180) {
                        int amount = (int) (num / 180);
                        float deg5 = num - (180 * amount);
                        firstNum = deg5;
                        return deg5;
                    }
                }
            }
        }
        return -1;
    }

    /**
     *
     * @param target- that target angle that is wanted to be acquired
     * @return the difference between the errror and the target amount
     */
    public float errorNum(float target) {
        float errorSol1=0;
        float errorSol2=0;

        errorSol1 =-1* (firstNum - target);
        errorSol2=-1*(target-firstNum);
        if (errorSol1 == 0 || errorSol2==0) {
            return 0;
        }
        if (errorSol1<errorSol2) {
            return errorSol1;
        }
        else {
            return errorSol2;
        }
    }

    /**
     *
     * @param theta1-first angle in degrees
     * @param theta2- second angle in degrees
     * @returns the sum of angles theta1 and theta2
     */
    public float addTheta(float theta1, float theta2, boolean conversion){
        float thetatotal=0;
        float thetafirst=0;
        float thetalast=0;
        thetafirst=converter(theta1, conversion);
        thetalast=converter(theta2,conversion);
        thetatotal=thetafirst+thetalast;
        return thetatotal;
    }

    /**
     *
     * @param theta1=the first angle that is going to be subtracted from it.
     * @param theta2= the second angle that is going to be subtracted from theta 1.
     * @return the  difference between angle one and angle two
     */
    public float minusTheta(float theta1, float theta2, boolean conversion){
        float thetaFirst=0;
        float thetaLast=0;
        float thetatotal=0;
        thetaFirst=converter(theta1,conversion);
        thetaLast=converter(theta2,conversion);
        thetatotal=thetaFirst-thetaLast;
        return thetatotal;
    }

    /**
     * Multiplies angle by a certain value that is given as a scalar of float type
     * @param theta-angle in degrees
     * @param scalar-any float value to be multiplied to theta
     * @return new multiplied angle value
     */
    /** public float ScalarTheta(float theta, float scalar){
     return theta*scalar;
     }
     public boolean convertNeeded(float theta, boolean convert){
     firstNum=theta;
     conversion=convert;
     return false;
     }
     */

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

}
