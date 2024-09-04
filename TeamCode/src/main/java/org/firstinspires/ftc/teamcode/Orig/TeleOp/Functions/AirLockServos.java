package org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class AirLockServos {

    private Servo airLock1, airLock2;
    private boolean airLockDownStatus, airLockRotateStatus;

    public AirLockServos(Servo _AL1, Servo _AL2) {
        airLock1 = _AL1;
        airLock2 = _AL2;
    }

    public void getDown() {
        airLock1.setPosition(1);
        airLockDownStatus = true;
    }
    public void getUp() {
        airLock1.setPosition(0);
        airLockDownStatus = false;
    }
    public void SwitchDown() {
        if(airLockDownStatus) {
            getUp();
        } else {
            getDown();
        }
    }
    double currentWaitTime =0;
    double currentTimeStamp =0;
    public void SwitchAndWaitDown(double x, double currentRuntime){
        if(currentWaitTime ==0|| currentTimeStamp + currentWaitTime <=currentRuntime){
            SwitchDown();
            currentTimeStamp =currentRuntime;
            currentWaitTime =x;
        }
    }
    public void rotateOpen() {
        airLock2.setPosition(1);
        airLockRotateStatus = true;
    }
    public void rotateClose() {
        airLock2.setPosition(0);
        airLockRotateStatus = false;
    }
    public void SwitchRotate() {
        if(airLockRotateStatus) {
            rotateClose();
        } else {
            rotateOpen();
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
