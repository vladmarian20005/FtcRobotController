package org.firstinspires.ftc.teamcode.Orig.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions.AirLockServos;
import org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions.BallServos;
import org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions.ClawArmEncoder;
import org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="RRTeleOpORIG", group = "ORIG")
public class RRTeleOp extends LinearOpMode {
    //Declare motors
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;
    private DcMotor clawArmMotor;

    //Declare servos
    private CRServo leftBall, rightBall;
    private Servo airLock1, airLock2;
    private Servo getPUp;
    private Servo closeClaw, rotateClaw;

    //Other declarations
    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 2;
    double movement;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private ElapsedTime runtime = new ElapsedTime();


    //declare classes
    private ArmEncoder controller;
    private BallServos ballServos;
    private ClawServos clawServos;
    private AirLockServos airLockServos;
    private ClawArmEncoder clawArmEncoder;

    GamepadCalc gamepadCalc;



    public void runOpMode() throws InterruptedException {
        int tickAdjustment = 100;

        //init motors
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("SL");
        armMotorRight = hardwareMap.dcMotor.get("SR");
        clawArmMotor = hardwareMap.dcMotor.get("CM");

        //init servos
        leftBall = hardwareMap.crservo.get("LB");
        rightBall = hardwareMap.crservo.get("RB");
        airLock1 = hardwareMap.servo.get("AL1");
        airLock2 = hardwareMap.servo.get("AL2");
        getPUp = hardwareMap.servo.get("PU");
        closeClaw = hardwareMap.servo.get("CS");
        rotateClaw = hardwareMap.servo.get("RS");

        //init classes
        controller = new ArmEncoder(armMotorLeft, armMotorRight);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);
        ballServos = new BallServos(leftBall,rightBall);
        clawServos = new ClawServos(closeClaw,rotateClaw);
        airLockServos = new AirLockServos(airLock1,airLock2);
        clawArmEncoder = new ClawArmEncoder(clawArmMotor);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        runtime.reset(); // Start game timer.
        clawServos.RotateLeft();
        if (isStopRequested()) return;
        while(opModeIsActive() && !isStopRequested()) {

            gamepadCalc.calculate();
            movement = gamepadCalc.getGamepad1().left_trigger - gamepadCalc.getGamepad1().right_trigger;

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );


            drive.update();
            drive.update();

            if(gamepad2.right_bumper)
            {
                drive.setPoseEstimate(PoseStorage.currentPose);
                telemetry.addData("Heading reseted to: ", PoseStorage.currentPose);
                telemetry.update();
            }
            if(gamepad2.dpad_up) {
                controller.goTo(4500,4500);
            }
            if(gamepad2.dpad_down) {
                controller.goTo(0,0);
            }
//            if(gamepad2.x) {
//                ballServos.startInv();
//            } else {
//                ballServos.Stop();
//            }
//            if(gamepad2.a) {
//               ballServos.Start();
//            } else {
//                ballServos.Stop();
//            }
            if(gamepad2.x) {
                ballServos.SwitchAndWaitContinuousInv(1,getRuntime());
            }
            if(gamepad2.a) {
                ballServos.SwitchAndWaitContinuous(1,getRuntime());
            }
            if(gamepad2.y) {
                clawServos.SwitchAndWaitClosed(1,getRuntime());
            }
            if(gamepad2.b) {
                clawServos.SwitchAndWaitRotate(1,getRuntime());
            }
            if(gamepad2.left_bumper) {
                airLockServos.rotateOpen();
            }
            if(gamepad2.right_bumper) {
                airLockServos.rotateClose();
            }
            if(gamepad2.left_stick_button) {
                getPUp.setPosition(1);
            }
            if(gamepad2.right_stick_button) {
                getPUp.setPosition(0);
            }
            if(gamepad1.dpad_left) {
                clawArmEncoder.goTo(100);
            }
            if(gamepad1.dpad_up) {
                clawArmEncoder.goTo(200);
            }
            if(gamepad1.dpad_right) {
                clawArmEncoder.goTo(300);
            }
            if(gamepad1.dpad_down) {
                clawArmEncoder.goTo(0);
            }

        }
    }

    //PID Control
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }


}
