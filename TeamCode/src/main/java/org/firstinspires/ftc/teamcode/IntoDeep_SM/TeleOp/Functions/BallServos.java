package org.firstinspires.ftc.teamcode.IntoDeep_SM.TeleOp.Functions;

import com.qualcomm.robotcore.hardware.CRServo;

public class BallServos {

    CRServo leftBall, rightBall;
    private boolean statusCr, statusInv;


    public BallServos(CRServo _LB, CRServo _RB) {
        leftBall = _LB;

        rightBall = _RB;
        statusCr = false;
        statusInv = false;
    }

    public void Start() {
        leftBall.setPower(1);
        rightBall.setPower(-1);
        statusCr=true;
        statusInv=false;
    }
    public void startInv() {
        leftBall.setPower(-1);
        rightBall.setPower(1);
        statusCr=false;
        statusInv=true;
    }
    public void Stop() {
        leftBall.setPower(0);
        rightBall.setPower(0);
    }

    public void Switch()
    {
        if(statusCr)
        {
            Stop();
        }
        else {
            Start();
        }
    }
    public void SwitchInv()
    {
        if(statusInv)
        {
            Stop();
        }
        else
        {
            startInv();
        }
    }

    double currentWaitTime =0;
    double currentTimeStamp =0;

    /**
     * This method checks if x seconds have passed, and if that's true => .
     */

    public void SwitchAndWaitContinuous(double x, double currentRuntime)
    {
        if(currentRuntime == 0 || currentTimeStamp + currentWaitTime <= currentRuntime) {
            Switch();
            currentTimeStamp = currentRuntime;
            currentWaitTime = x;
        }
    }
    public void SwitchAndWaitContinuousInv(double x, double currentRuntime)
    {
        if(currentRuntime == 0 || currentTimeStamp + currentWaitTime <= currentRuntime) {
            SwitchInv();
            currentTimeStamp = currentRuntime;
            currentWaitTime = x;
        }
    }
}
