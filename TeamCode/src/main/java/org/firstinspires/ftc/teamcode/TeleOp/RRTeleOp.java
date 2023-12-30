package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.PIDController;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="RRTeleOp", group = "GAME")
public class RRTeleOp extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor aspiratorMotor;

    Vacuum vacuum;
        private DcMotor armMotorLeft, armMotorRight;
//    private Servo clawServo, topServo;
    private PIDController controller;
//    private Move move;
    private Rotate rotate;
//    private ClawServos clawServos;
//    private ArmEncoder armEncoder;
//    private TopServos topServos;
//    private Servo servo;
    private boolean status1;
    int armLevel = 0;
    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 2;
    double movement;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    String armCurrentDirection = "up";
    private ElapsedTime runtime = new ElapsedTime();
    final double END_GAME = 90.0;
    final double FIFTEEN_SECONDS = 105.0;
    final double FIVE_SECONDS = 115.0;
    boolean secondFifteen = false;
    boolean secondEnd = false;
    boolean secondFive = false;
    int count = 400;
    boolean goDown = true;
    Gamepad.RumbleEffect customRumbleEffectFive, customRumbleEffectFifteen, customRumbleEffectEnd;
    double speed = .7;

    GamepadCalc gamepadCalc;

    @Override
    public void runOpMode() throws InterruptedException {

        customRumbleEffectFive = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .build();

        customRumbleEffectFifteen = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 600)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 600)  //  Rumble left motor 100% for 250 mSec
                .build();

        customRumbleEffectEnd = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 700)  //  Rumble right motor 100% for 500 mSec
                .build();

        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        aspiratorMotor = hardwareMap.dcMotor.get("AS");
        armMotorLeft = hardwareMap.dcMotor.get("SL");
        armMotorRight = hardwareMap.dcMotor.get("SR");
//        topServo = hardwareMap.servo.get("TS");
//        clawServo = hardwareMap.servo.get("CS");
//        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
//        clawServos = new ClawServos(clawServo);
//        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
//        topServos = new TopServos(topServo);
        controller = new PIDController(armMotorLeft, armMotorRight);
        vacuum = new Vacuum(aspiratorMotor);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        servosUp(topServo);

        waitForStart();

        runtime.reset(); // Start game timer.

        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {

            //Watch the runtime timer, and run the custom rumble when we hit half-time.
            //Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
            if ((runtime.seconds() > FIFTEEN_SECONDS) && !secondFifteen)  {
                gamepad1.runRumbleEffect(customRumbleEffectFifteen);
                gamepad2.runRumbleEffect(customRumbleEffectFifteen);
                secondFifteen =true;
            }
            else if ((runtime.seconds() > END_GAME) && !secondEnd)  {
                gamepad1.runRumbleEffect(customRumbleEffectEnd);
                gamepad2.runRumbleEffect(customRumbleEffectEnd);
                secondEnd =true;
            }
            else if ((runtime.seconds() > FIVE_SECONDS) && !secondFive)  {
                gamepad1.runRumbleEffect(customRumbleEffectFive);
                gamepad2.runRumbleEffect(customRumbleEffectFive);
                secondFive =true;
            }


            gamepadCalc.calculate();
            movement = gamepadCalc.getGamepad1().left_trigger - gamepadCalc.getGamepad1().right_trigger;

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y*speed,
                    -gamepad1.left_stick_x*speed
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x*speed
                    )
            );
            drive.update();

            if(gamepad1.dpad_left)
            {
                rotate.RotateFull(1);
            }
            if(gamepad1.dpad_right)
            {
                rotate.RotateFull(2);
            }

            if(gamepad1.right_bumper)
            {
                drive.setPoseEstimate(PoseStorage.currentPose);
                telemetry.addData("Heading reseted to: ", PoseStorage.currentPose);
            }

            if (gamepad1.x) {
                vacuum.Start();
            }
            if (gamepad1.y){
                vacuum.Stop();
            }
//            if (gamepad1.dpad_up){
//                controller.goTo(500, 500);
//            }
//
//            if (gamepad1.dpad_down){
//                controller.goTo(0, 0);
//            }
//            {
//                telemetry.addData("count:", count);
//                telemetry.update();
//                servosUp(topServo);
//                sleep(500);
//                if (goDown == true) {
//                    switch (count) {
//                        case 500:
//                            armEncoder.goTo(500, 500);
//                            count -= 100;
//                            break;
//                        case 400:
//                            armEncoder.goTo(400, 400);
//                            count -= 100;
//                            break;
//                        case 300:
//                            armEncoder.goTo(300, 300);
//                            count -= 100;
//                            break;
//                        case 200:
//                            armEncoder.goTo(200, 200);
//                            count -= 100;
//                            break;
//                        default:
//                            break;
//                    }
//                    goDown = false;
//                }
//            }
//            if(gamepad2.left_bumper)
//            {
//                goDown = true;
//            }
//            if(gamepad2.a)
//            {
//                count = 500;
//                telemetry.addData("Count reseted to: ", count);
//                telemetry.update();
//            }
//            if(gamepad2.x)
//            {
//                clawServos.SwitchAndWait(1,getRuntime());
//            }
//
//            if(gamepad2.y)
//            {
//                servosDown(topServo);
//            }
//            if(gamepad2.b)
//            {
//                servosUp(topServo);
//            }
//
//            if (gamepad2.dpad_up) // Arm Up
//            {
//                armCurrentDirection = "up";
//                armEncoder.goTo(3000, 3000);
//            }
//            if (gamepad2.dpad_down) //Arm Down
//            {
//                armCurrentDirection = "down";
//                servosUp(topServo);
//                sleep(500);
//                armEncoder.goTo(0, 0);
//                closeServo(clawServo);
//            }
//            if(gamepad2.dpad_left) // Arm level 1
//            {
//                armCurrentDirection = "up";
//                armEncoder.goTo(1300, 1300);
//            }
//            if(gamepad2.dpad_right) // Level 2
//            {
//                armCurrentDirection = "up";
//                armEncoder.goTo(2000, 2000);
//            }
        }
//            if (armCurrentDirection.equals("down")) {
//                armMotorLeft.setPower(0);
//                armMotorRight.setPower(0);
//                armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
    }


    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
    private void openServo(Servo _LS)
    {
        _LS.setPosition(1);
    }
    private void closeServo(Servo _LS)
    {
        _LS.setPosition(0);
    }
    private void servosUp(Servo topLeftServo)
    {
        topLeftServo.setPosition(0);
    }
    private void servosDown(Servo topLeftServo)
    {
        topLeftServo.setPosition(1);
    }


}