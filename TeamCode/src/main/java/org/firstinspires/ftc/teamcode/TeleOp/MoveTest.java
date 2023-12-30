package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.Move2;

//@Disabled
@TeleOp(name = "DriveTest", group = "TEST")
public class MoveTest extends OpMode {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private Move2 move;

    @Override
    public void init() {
        /**
         * Initializing the motors
         */
        leftMotor = hardwareMap.dcMotor.get("LM");
        rightMotor = hardwareMap.dcMotor.get("RM");
        leftMotorBack = hardwareMap.dcMotor.get("LMB");
        rightMotorBack = hardwareMap.dcMotor.get("RMB");

        move = new Move2(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_down)
        {
            move.MoveFull(2);
        }
        else if(gamepad1.dpad_up)
        {
            move.MoveFull(1);
        }
        else if(gamepad1.dpad_left)
        {
            move.MoveFull(3);
        }
        else if(gamepad1.dpad_right)
        {
            move.MoveFull(4);
        }
        else move.MoveStop();


        if(gamepad1.a)
        {
            move.MoveOne(1);
        }
        else if(gamepad1.b)
        {
            move.MoveOne(2);
        }
        else if(gamepad1.x)
        {
            move.MoveOne(3);
        }
        else if(gamepad1.y)
        {
            move.MoveOne(4);
        }
        else move.MoveStop();


        if(gamepad1.left_bumper)
            move.Rotate(1);
        else if(gamepad1.right_bumper)
            move.Rotate(2);
        else move.MoveStop();
    }
}
