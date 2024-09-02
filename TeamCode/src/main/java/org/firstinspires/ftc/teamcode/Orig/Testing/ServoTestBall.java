package org.firstinspires.ftc.teamcode.Orig.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "ServoTest", group = "ORIG- TEST")
public class ServoTestBall extends LinearOpMode {
    private CRServo leftBall, rightBall;

    public void runOpMode() throws InterruptedException{
        leftBall = hardwareMap.crservo.get("LB");
        rightBall = hardwareMap.crservo.get("RB");

        waitForStart();

        if(gamepad1.x) {
            servoPower(leftBall,rightBall);
        } else {
            stopServo(leftBall,rightBall);
        }

    }

    private void servoPower(CRServo _leftBall, CRServo _rightBall) {
        _leftBall.setPower(1);
        _rightBall.setPower(-1);
    }
    private void stopServo(CRServo _leftBall, CRServo _rightBall) {
        _leftBall.setPower(0);
        _rightBall.setPower(0);
    }
}
