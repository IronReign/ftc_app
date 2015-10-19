package com.ironempire.util;
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

    public Theta(float angle, boolean convert) {
        firstNum = angle;
        conversion = convert;
    }

    public float converter(float num, boolean convert) {
        if (convert == true) {
            num = (float) ((num * Math.PI) / 180);
            firstNum = num;
            return num;
        }
        if ((convert == false) && (num <= 180 && num >= -180)) {
            float deg = num;
            firstNum = deg;
            return deg;
        } else {
            if (convert == false) {

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
}