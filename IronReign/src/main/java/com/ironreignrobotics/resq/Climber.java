package com.ironreignrobotics.resq;

import com.qualcomm.robotcore.hardware.*;

import org.swerverobotics.library.*;
import org.swerverobotics.library.SwerveUtil;
import org.swerverobotics.library.interfaces.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.ironempire.util.*;

enum ThreadState {//To support implementation of state machines using "switch" statements, this
    //"enum" gives names to the "cases" included in the "switch" statements. The
    // names may, of course, be whatever the programmers consider meaningful.
    STATE_ZERO,
    STATE_ONE,
    STATE_STOP
//    STATE_TWO,
//    STATE_THREE
}

class Climber implements Runnable {
    volatile DcMotor motorClimber = null;

    volatile ThreadState climberThreadState;
    long hoistThreadInterval = 100;//Number of milliseconds between the starts of successive
    //iterations
    long iterationStart;
    public Climber(DcMotor motorIn){ //Constructor
        climberThreadState = ThreadState.STATE_STOP; //A state machine must have an initial state
        motorClimber = motorIn;
    }
    public void run() {

        //while (!Thread.interrupted()) { //A call to "interrupt()" is what stops this thread
            iterationStart = System.currentTimeMillis();
            switch (climberThreadState) {

                case STATE_ZERO: {
                    motorClimber.setTargetPosition(-750);
                    motorClimber.setPower(-.5);
                    if(motorClimber.getCurrentPosition() < -740)
                        climberThreadState = ThreadState.STATE_ONE;
                    break;
                }
                case STATE_ONE: {
                    motorClimber.setTargetPosition(0);
                    motorClimber.setPower(.5);
                    if(motorClimber.getCurrentPosition() > -10)
                        climberThreadState = ThreadState.STATE_STOP;
                    break;
                }
                case STATE_STOP:{
                    if(motorClimber.getPower() != 0.0)
                        motorClimber.setPower(0);
                    break;
                }
                default: {
                    break;
                }
            }
            /*try { //When the work of each iteration is done, "sleep" until time to start the next one.
                //The "sleep" time required depends on the duration of the iteration just ended.
                Thread.sleep(hoistThreadInterval + Math.max(-hoistThreadInterval,
                        (iterationStart - System.currentTimeMillis())));
            } catch (InterruptedException e) {
                break;  //Exit the thread's main loop
                }
            */
    }


    //}
    void stroke()
    {
        climberThreadState = ThreadState.STATE_ZERO;
    }

}