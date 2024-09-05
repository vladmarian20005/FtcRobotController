package org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServos {

    private Servo closeServo, rotateServo;
    private boolean closeStatus, rotateStatus;


    public ClawServos(Servo _CS, Servo _RS) {
        closeServo = _CS;
        rotateServo = _RS;
        closeStatus = false;
        rotateStatus = false;
    }

    public void Close() {
        closeServo.setPosition(1);
        closeStatus = true;
    }
    public void Open() {
        closeServo.setPosition(0);
        closeStatus = false;
    }
    public void RotateLeft() {
        rotateServo.setPosition(1);
        rotateStatus = true;
    }
    public void RotateCenter() {
        rotateServo.setPosition(0.35);
        rotateStatus = false;
    }
    public void SwitchRotate() {
        if(rotateStatus) {
            RotateCenter();
        } else {
            RotateLeft();
        }
    }
    public void SwitchClosed() {
        if(closeStatus) {
            Open();
        } else {
            Close();
        }
    }
    double currentWaitTime =0;
    double currentTimeStamp =0;

    public void SwitchAndWaitClosed(double x, double currentRuntime){
        if(currentWaitTime ==0|| currentTimeStamp + currentWaitTime <=currentRuntime){
            SwitchClosed();
            currentTimeStamp =currentRuntime;
            currentWaitTime =x;
        }
    }
    public void SwitchAndWaitRotate(double x, double currentRuntime) {
        if(currentWaitTime == 0 || currentTimeStamp + currentWaitTime <= currentRuntime) {
            SwitchRotate();
            currentTimeStamp = currentRuntime;
            currentWaitTime = x;
        }
    }
}
