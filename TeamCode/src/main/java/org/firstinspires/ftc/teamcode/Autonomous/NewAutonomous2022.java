/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.AccelerationReader;

import org.firstinspires.ftc.teamcode.Functions.EncoderMove;
import org.firstinspires.ftc.teamcode.Functions.Arm;

import org.firstinspires.ftc.teamcode.Functions.ArmServos;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.EncoderMove;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "NewAutonomous2022", group = "Concept")
@Disabled
public class NewAutonomous2022 extends LinearOpMode {

    /**
     * Initialize the motors.
     */
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotorBack;
    private DcMotor rightMotorBack;
    private MoveAutocorrect2 AutoCorrection;
    private RotationDetector rotationDetector;
    private Move move;
    private Rotate rotate;
    private EncoderMove encoderMove;
    private Servo L1Servo;
    private Servo L2Servo;
    private ArmServos armServos;
    private Arm arm;
    private DcMotorEx armMotorLeft, armMotorRight;


    /**
     * Initialize the variables that store the current position of the encoders.
     */

    private int leftPos;
    private int rightPos;
    private int leftBackPos;
    private int rightBackPos;



    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "FL");
        rightMotor = hardwareMap.get(DcMotor.class, "FR");
        leftMotorBack = hardwareMap.get(DcMotor.class, "BL");
        rightMotorBack = hardwareMap.get(DcMotor.class, "BR");

        L1Servo = hardwareMap.servo.get("L1S");
        L2Servo = hardwareMap.servo.get("L2S");
        AutoCorrection = new MoveAutocorrect2(rotationDetector,move,rotate);
        move = new Move(leftMotor, rightMotor, leftMotor, rightMotor);
        rotate = new Rotate(leftMotor, rightMotor, leftMotor, rightMotor);
        encoderMove = new EncoderMove(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
      //  armServos = new ArmServos(L1Servo, L2Servo);
        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");


        waitForStart();
        arm.Start(-1);
        sleep(2000);
       // armServos.Level2Up();
        sleep(2000);
        arm.Stop();
        sleep(3000);

        arm.Start(-1);
        sleep(2000);
      //  armServos.Level2Down();
        sleep(2000);
        arm.Stop();


        AutoCorrection = new MoveAutocorrect2(rotationDetector,move,rotate);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);

        /**
         * Resets the encoders to 0 position.
         */
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /**
         * Reverse the motor so the drive wheels don't spin in opposite direction.
         */
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
         * Initialize the encoders position to 0.
         */
        leftPos = 0;
        rightPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;

        waitForStart();

        sleep(2000);

        /**
         * This will make robot move forward & stop.
         */
        drive(3000, 3000, 3000, 3000, 0.5);
        sleep(1000);


//        /**
//         * This will make robot turn right & stop.
//         */
//        drive(1000, -1000, 0.25);
    }

    // private void drive(int leftTarget, int rightTarget, int leftBackTarget, int rightBackTarget, double speed)
    private void drive(int rightBackTarget, int leftBackTarget, int leftTarget, int rightTarget, double speed){

        leftPos += leftTarget;
        rightPos += rightTarget;
        leftBackPos += leftBackTarget;
        rightBackPos += rightBackTarget;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        leftMotorBack.setTargetPosition(leftPos);
        rightMotorBack.setTargetPosition(rightPos);
        leftMotor.setTargetPosition(leftBackPos);
        rightMotor.setTargetPosition(rightBackPos);

        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotorBack.setPower(speed);
        rightMotorBack.setPower(speed);
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy() && leftMotorBack.isBusy() && rightMotorBack.isBusy()){
            idle();
//            Autocorrect();
        }
    }
}