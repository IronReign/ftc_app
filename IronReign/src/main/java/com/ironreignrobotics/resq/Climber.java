package com.ironreignrobotics.resq;

import com.qualcomm.robotcore.hardware.*;

enum ThreadState {//To support implementation of state machines using "switch" statements, this
    //"enum" gives names to the "cases" included in the "switch" statements. The
    // names may, of course, be whatever the programmers consider meaningful.
    STATE_EXTENDING,
    STATE_RETRACTING,
    STATE_RESTING

}

class Climber implements Runnable {
    volatile DcMotor motorClimber = null;

    volatile ThreadState climberThreadState;
    long climberThreadInterval = 100;//Number of milliseconds between the starts of successive
    //iterations
    long iterationStart;
    public Climber(DcMotor motorIn){ //Constructor
        climberThreadState = ThreadState.STATE_RESTING; //A state machine must have an initial state
        motorClimber = motorIn;
    }
    public void run() {
        //it's not working as a thread, so commenting some stuff out and calling this once per main loop
        //while (!Thread.interrupted()) { //A call to "interrupt()" is what stops this thread
            iterationStart = System.currentTimeMillis();
            switch (climberThreadState) {

                case STATE_EXTENDING: { //extend

                    if (motorClimber.getTargetPosition()!= -1600) //set target once per state transition
                    {
                    motorClimber.setTargetPosition(-1600);
                    motorClimber.setPower(-.5);
                    }
                    if(motorClimber.getCurrentPosition() < -1590)
                        climberThreadState = ThreadState.STATE_RETRACTING;
                    break;
                }
                case STATE_RETRACTING: { //retract
                    if (motorClimber.getTargetPosition()!= 0) { //set target once per state transition
                        motorClimber.setTargetPosition(0);
                        motorClimber.setPower(.75);
                    }
                    if(motorClimber.getCurrentPosition() > -10)
                        climberThreadState = ThreadState.STATE_RESTING;
                    break;
                }
                case STATE_RESTING:{
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
        climberThreadState = ThreadState.STATE_EXTENDING;
    }

}