package org.firstinspires.ftc.teamcode.IntoDeep_SM.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="RRTeleOpIntoDeep", group = "IntoDeep_SM")
public class RRTeleOp extends LinearOpMode {
    //Declare motors

    // Drive motors (GoBilda Yellow Jacket)
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;

    // GoBilda motor constants
    private static final double GOBILDA_TICKS_PER_REV = 537.7; // For 19.2:1 Yellow Jacket
    private static final double WHEEL_DIAMETER_MM = 96.0;  // 96mm GoBilda Mecanum wheel
    private static final double DRIVE_GEAR_RATIO = 1.0;    // Direct drive

    // Viper slide extension motors
    private DcMotorEx armMotorLeft, armMotorRight;

    // Viper slide rotation motors
    private DcMotorEx rotateMotorLeft, rotateMotorRight;

    // Intake servos
    private Servo leftAxleServo, rightAxleServo;      // Servos for rotating the intake axle
    private Servo leftGeckoServo, rightGeckoServo;    // Servos for Gecko wheels


    // PID and timing variables (consolidated)
    private ElapsedTime runtime = new ElapsedTime();
    private double movement;

    // Initialize controller classes
    private ArmEncoder controller;
    private SampleMecanumDrive drive;
    private GamepadCalc gamepadCalc;


    // Constants for viper slide positions
    private static final int SLIDES_LOW_POSITION = 0;
    private static final int SLIDES_MEDIUM_POSITION = 2250;  // Adjust based on your needs
    private static final int SLIDES_HIGH_POSITION = 4500;    // Adjust based on your needs

    // Constants for rotation positions (in ticks)
    private static final int ROTATION_HORIZONTAL = 0;
    private static final int ROTATION_45_DEGREES = 384;    // Adjust based on your gear ratio
    private static final int ROTATION_VERTICAL = 768;      // Adjust based on your gear ratio

    // Constants for servo positions
    private static final double AXLE_INTAKE_POSITION = 0.0;     // Position for intaking
    private static final double AXLE_DEPOSIT_POSITION = 1.0;    // Position for depositing
    private static final double GECKO_WHEEL_STOP = 0.5;         // Neutral position
    private static final double GECKO_WHEEL_INTAKE = 1.0;       // Intake direction
    private static final double GECKO_WHEEL_OUTTAKE = 0.0;      // Outtake direction

    // PID Constants for viper slides
    private static final double SLIDES_P = 0.005;
    private static final double SLIDES_I = 0.0;
    private static final double SLIDES_D = 0.0;

    // PID Constants for rotation
    private static final double ROTATION_P = 0.005;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.0;

    // PID variables
    private ElapsedTime slidesTimer = new ElapsedTime();
    private ElapsedTime rotationTimer = new ElapsedTime();
    private double lastSlidesError = 0;
    private double lastRotationError = 0;
    private double slidesIntegralSum = 0;
    private double rotationIntegralSum = 0;

    // Safety constants
    private static final int SLIDES_MAX_POSITION = 4700;  // Absolute maximum extension
    private static final int SLIDES_MIN_POSITION = -10;   // Allow slight negative for zero calibration
    private static final double SLIDES_MAX_POWER = 1.0;   // Maximum allowed power
    private static final double ROTATION_MAX_POWER = 0.7; // Maximum rotation power

    // Motor current monitoring thresholds (in amps)
    private static final double CURRENT_LIMIT_SLIDES = 2.5;
    private static final double CURRENT_LIMIT_ROTATION = 2.0;

    // Coordination constants
    private static final int SAFE_ROTATION_EXTENSION = 1000; // Minimum extension for rotation
    private static final int MAX_EXTENSION_AT_ANGLE = 3000;  // Maximum extension when rotated

    // State tracking
    private boolean isOverCurrentProtected = false;
    private double lastSlidePower = 0;
    private double lastRotationPower = 0;


    // State machine enums
    private enum SlideState {
        IDLE,
        MANUAL_CONTROL,
        MOVING_TO_POSITION,
        ERROR
    }

    private enum RotationState {
        IDLE,
        MANUAL_CONTROL,
        MOVING_TO_POSITION,
        ERROR
    }

    private enum IntakeState {
        IDLE,
        INTAKING,
        OUTTAKING,
        ERROR
    }

    // State tracking
    private SlideState slideState = SlideState.IDLE;
    private RotationState rotationState = RotationState.IDLE;
    private IntakeState intakeState = IntakeState.IDLE;
    private int targetSlidePosition = 0;
    private int targetRotationPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        int tickAdjustment = 100;

        // Initialize hardware
        initializeHardware();

        // Configure motor behaviors
        setupMotors();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            gamepadCalc.calculate();
            movement = gamepadCalc.getGamepad1().left_trigger - gamepadCalc.getGamepad1().right_trigger;

            handleDrive(drive);
            handleViperSlides();
            handleRotation();
            handleIntake();
            updateTelemetry();
            drive.update();

            if(gamepad2.right_bumper) {
                drive.setPoseEstimate(PoseStorage.currentPose);
                telemetry.addData("Heading reseted to: ", PoseStorage.currentPose);
                telemetry.update();
            }
        }
    }

    private void initializeHardware() {
        // Initialize drive motors
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        // Initialize viper slide motors
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "SL");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "SR");

        // Initialize rotation motors
        rotateMotorLeft = hardwareMap.get(DcMotorEx.class, "RL");
        rotateMotorRight = hardwareMap.get(DcMotorEx.class, "RR");

        // Initialize intake servos
//        leftAxleServo = hardwareMap.servo.get("LA");    // Left Axle servo
//        rightAxleServo = hardwareMap.servo.get("RA");   // Right Axle servo
//        leftGeckoServo = hardwareMap.servo.get("LG");   // Left Gecko wheel
//        rightGeckoServo = hardwareMap.servo.get("RG");  // Right Gecko wheel

        // Initialize controller classes
        controller = new ArmEncoder(armMotorLeft, armMotorRight);
        drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);
    }

    private void setupMotors() {
        // Drive motors setup - GoBilda configuration
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set GoBilda motor directions
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);

        // Viper slide extension motors setup
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // Rotation motors setup
        rotateMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMotorRight.setDirection(DcMotor.Direction.REVERSE);  // Set opposite direction
    }

    private void handleDrive(SampleMecanumDrive drive) {
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
    }

    private void handleViperSlides() {
        // Manual control with gamepad2 left stick
        double slidePower = -gamepad2.left_stick_y;
        boolean isManualControl = Math.abs(slidePower) > 0.1;
        boolean isPresetRequested = gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down;
        int currentPosition = (armMotorLeft.getCurrentPosition() + armMotorRight.getCurrentPosition()) / 2;

        // Check current limits first
//        if (checkCurrentLimits()) {
//            slideState = SlideState.ERROR;
//            stopSlides();
//            return;
//        }

        // Check for overcurrent protection
        if (isOverCurrentProtected) {
            stopSlides();
            telemetry.addData("WARNING", "Slide motors current limit exceeded!");
            return;
        }

        // Position limits
        if (currentPosition > SLIDES_MAX_POSITION && slidePower > 0) {
            slidePower = 0;
            telemetry.addData("WARNING", "Maximum extension reached!");
        }
        if (currentPosition < SLIDES_MIN_POSITION && slidePower < 0) {
            slidePower = 0;
            telemetry.addData("WARNING", "Minimum position reached!");
        }


        // State machine for slides
        switch (slideState) {
            case IDLE:
                if (isManualControl) {
                    slideState = SlideState.MANUAL_CONTROL;
                } else if (isPresetRequested) {
                    slideState = SlideState.MOVING_TO_POSITION;
                    if (gamepad2.dpad_up) targetSlidePosition = SLIDES_HIGH_POSITION;
                    if (gamepad2.dpad_right) targetSlidePosition = SLIDES_MEDIUM_POSITION;
                    if (gamepad2.dpad_down) targetSlidePosition = SLIDES_LOW_POSITION;
                }
                break;

            case MANUAL_CONTROL:
                if (!isManualControl) {
                    slideState = SlideState.IDLE;
                    stopSlides();
                } else {
                    double smoothedPower = getSmoothedSlidePower(slidePower);
                    applySlidePower(smoothedPower);
                }
                break;

            case MOVING_TO_POSITION:
                if (isManualControl) {
                    slideState = SlideState.MANUAL_CONTROL;
                } else if (isAtTargetPosition()) {
                    slideState = SlideState.IDLE;
                    stopSlides();
                } else {
                    moveToTargetPosition();
                }
                break;

            case ERROR:
//                if (!checkCurrentLimits()) {
//                    slideState = SlideState.IDLE;
//                }
                break;
        }

        // Position telemetry can stay if needed
        if(armMotorLeft.getCurrentPosition()==5000 && armMotorRight.getCurrentPosition()==5000) {
            telemetry.addData("Slider Pos: ", "ok");
        }
    }

    private void handleRotation() {
        double rotatePower = -gamepad2.right_stick_y * ROTATION_MAX_POWER;
        boolean isManualControl = Math.abs(rotatePower) > 0.1;
        boolean isPresetRequested = gamepad2.left_bumper || gamepad2.right_bumper || gamepad2.left_trigger > 0.5;
        int currentRotation = (rotateMotorLeft.getCurrentPosition() + rotateMotorRight.getCurrentPosition()) / 2;
        int slidePosition = (armMotorLeft.getCurrentPosition() + armMotorRight.getCurrentPosition()) / 2;

        // Check current limits first
//        if (checkRotationCurrentLimits()) {
//            rotationState = RotationState.ERROR;
//            stopRotation();
//            telemetry.addData("WARNING", "Rotation motors current limit exceeded!");
//            return;
//        }

        // Check for overcurrent protection
        if (isOverCurrentProtected) {
            stopRotation();
            telemetry.addData("WARNING", "Rotation motors current limit exceeded!");
            return;
        }

        // Safety check for rotation when slides are retracted
        if (slidePosition < SAFE_ROTATION_EXTENSION && currentRotation < ROTATION_HORIZONTAL
                && Math.abs(rotatePower) > 0.1) {
            telemetry.addData("WARNING", "Extend slides before rotating!");
            rotatePower = 0;
        }

        // Prevent rotation past limits
        if ((currentRotation >= ROTATION_VERTICAL && rotatePower > 0) ||
                (currentRotation <= ROTATION_HORIZONTAL && rotatePower < 0)) {
            rotatePower = 0;
            telemetry.addData("WARNING", "Rotation limit reached!");
        }

        // State machine for rotation
        switch (rotationState) {
            case IDLE:
                stopRotation();
                if (isManualControl) {
                    rotationState = RotationState.MANUAL_CONTROL;
                } else if (isPresetRequested) {
                    rotationState = RotationState.MOVING_TO_POSITION;
                    if (gamepad2.left_bumper) targetRotationPosition = ROTATION_HORIZONTAL;
                    if (gamepad2.right_bumper) targetRotationPosition = ROTATION_VERTICAL;
                    if (gamepad2.left_trigger > 0.5) targetRotationPosition = ROTATION_45_DEGREES;
                }
                break;

            case MANUAL_CONTROL:
                if (!isManualControl) {
                    rotationState = RotationState.IDLE;
                    stopRotation();
                } else {
                    double smoothedPower = getSmoothedRotationPower(rotatePower);
                    applyRotationPower(smoothedPower);
                }
                break;

            case MOVING_TO_POSITION:
                if (isManualControl) {
                    rotationState = RotationState.MANUAL_CONTROL;
                } else if (isAtRotationTarget()) {
                    rotationState = RotationState.IDLE;
                    stopRotation();
                } else {
                    rotateViperSlidesTo(targetRotationPosition);
                }
                break;

            case ERROR:
//                if (!checkRotationCurrentLimits()) {
//                    rotationState = RotationState.IDLE;
//                }
                break;
        }
    }

    private void handleIntake() {
        // Axle rotation control using gamepad2 buttons
        if (gamepad2.x) {  // Intake position
//            leftAxleServo.setPosition(AXLE_INTAKE_POSITION);
//            rightAxleServo.setPosition(1 - AXLE_INTAKE_POSITION);  // Reverse for opposite side
        } else if (gamepad2.y) {  // Deposit position
//            leftAxleServo.setPosition(AXLE_DEPOSIT_POSITION);
//            rightAxleServo.setPosition(1 - AXLE_DEPOSIT_POSITION);  // Reverse for opposite side
        }

        // Gecko wheel control using right trigger for intake, left trigger for outtake
        if (gamepad2.right_trigger > 0.1) {  // Intake
//            leftGeckoServo.setPosition(GECKO_WHEEL_INTAKE);
//            rightGeckoServo.setPosition(1 - GECKO_WHEEL_INTAKE);  // Reverse for opposite direction
        } else if (gamepad2.left_trigger > 0.1) {  // Outtake
//            leftGeckoServo.setPosition(GECKO_WHEEL_OUTTAKE);
//            rightGeckoServo.setPosition(1 - GECKO_WHEEL_OUTTAKE);  // Reverse for opposite direction
        } else {  // Stop
//            leftGeckoServo.setPosition(GECKO_WHEEL_STOP);
//            rightGeckoServo.setPosition(GECKO_WHEEL_STOP);
        }
    }

    private void moveViperSlidesTo(int targetPosition) {
        // Safety bounds check
        targetPosition = Range.clip(targetPosition, SLIDES_MIN_POSITION, SLIDES_MAX_POSITION);

        // Check if rotation angle allows this extension
        int rotationPosition = (rotateMotorLeft.getCurrentPosition() + rotateMotorRight.getCurrentPosition()) / 2;
        if (rotationPosition > ROTATION_45_DEGREES && targetPosition > MAX_EXTENSION_AT_ANGLE) {
            targetPosition = MAX_EXTENSION_AT_ANGLE;
            telemetry.addData("WARNING", "Extension limited due to rotation angle!");
        }

        int currentPosition = (armMotorLeft.getCurrentPosition() + armMotorRight.getCurrentPosition()) / 2;
        double power = calculatePID(targetPosition, currentPosition, SLIDES_P, SLIDES_I, SLIDES_D,
                slidesTimer, lastSlidesError, slidesIntegralSum);

        // Apply power with safety limit
        power = Range.clip(power, -SLIDES_MAX_POWER, SLIDES_MAX_POWER);
        if (!isOverCurrentProtected) {
            armMotorLeft.setPower(power);
            armMotorRight.setPower(power);
        }
    }

    private void rotateViperSlidesTo(int targetPosition) {
        // Safety bounds check
        targetPosition = Range.clip(targetPosition, ROTATION_HORIZONTAL, ROTATION_VERTICAL);

        int currentPosition = (rotateMotorLeft.getCurrentPosition() + rotateMotorRight.getCurrentPosition()) / 2;
        double power = calculatePID(targetPosition, currentPosition, ROTATION_P, ROTATION_I, ROTATION_D,
                rotationTimer, lastRotationError, rotationIntegralSum);

        // Update PID variables
        lastRotationError = currentPosition - targetPosition;

        // Apply power with safety limit
        power = Range.clip(power, -ROTATION_MAX_POWER, ROTATION_MAX_POWER);
        if (!isOverCurrentProtected) {
            applyRotationPower(power);
        }
    }

    private double calculatePID(double reference, double state, double kP, double kI, double kD,
                                ElapsedTime timer, double lastError, double integralSum) {
        double error = reference - state;
        double deltaTime = timer.seconds();
        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return Range.clip(output, -1, 1);  // Clamp output between -1 and 1
    }

//    private boolean checkCurrentLimits() {
//        double leftCurrent = armMotorLeft.getCurrent();  // Just use getCurrent() directly
//        double rightCurrent = armMotorRight.getCurrent();
//        return leftCurrent > CURRENT_LIMIT_SLIDES || rightCurrent > CURRENT_LIMIT_SLIDES;
//    }
//
//    private boolean checkRotationCurrentLimits() {
//        double leftCurrent = rotateMotorLeft.getCurrent();
//        double rightCurrent = rotateMotorRight.getCurrent();
//        return leftCurrent > CURRENT_LIMIT_ROTATION || rightCurrent > CURRENT_LIMIT_ROTATION;
//    }

    private void stopSlides() {
        armMotorLeft.setPower(0);
        armMotorRight.setPower(0);
    }

    private double getSmoothedSlidePower(double rawPower) {
        double safePower = Range.clip(rawPower, -SLIDES_MAX_POWER, SLIDES_MAX_POWER);
        if (Math.abs(safePower - lastSlidePower) > 0.5) {
            safePower = (safePower + lastSlidePower) / 2;
        }
        lastSlidePower = safePower;
        return safePower;
    }

    private void applySlidePower(double power) {
        // Single place where power is applied to slides
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
    }

    private boolean isAtTargetPosition() {
        int currentPosition = (armMotorLeft.getCurrentPosition() +
                armMotorRight.getCurrentPosition()) / 2;
        return Math.abs(currentPosition - targetSlidePosition) < 10;
    }

    private void moveToTargetPosition() {
        int currentPosition = (armMotorLeft.getCurrentPosition() +
                armMotorRight.getCurrentPosition()) / 2;
        double power = calculatePID(targetSlidePosition, currentPosition,
                SLIDES_P, SLIDES_I, SLIDES_D,
                slidesTimer, lastSlidesError, slidesIntegralSum);
        applySlidePower(Range.clip(power, -SLIDES_MAX_POWER, SLIDES_MAX_POWER));
    }

    private void stopRotation() {
        rotateMotorLeft.setPower(0);
        rotateMotorRight.setPower(0);
    }

    private void applyRotationPower(double power) {
        rotateMotorLeft.setPower(power);
        rotateMotorRight.setPower(power);
    }

    private double getSmoothedRotationPower(double rawPower) {
        double safePower = Range.clip(rawPower, -ROTATION_MAX_POWER, ROTATION_MAX_POWER);
        if (Math.abs(safePower - lastRotationPower) > 0.3) {
            safePower = (safePower + lastRotationPower) / 2;
        }
        lastRotationPower = safePower;
        return safePower;
    }

    private boolean isAtRotationTarget() {
        int currentPosition = (rotateMotorLeft.getCurrentPosition() +
                rotateMotorRight.getCurrentPosition()) / 2;
        return Math.abs(currentPosition - targetRotationPosition) < 10;
    }

    private void updateTelemetry() {
        telemetry.addData("Left Slide Position", armMotorLeft.getCurrentPosition());
        telemetry.addData("Right Slide Position", armMotorRight.getCurrentPosition());
        telemetry.addData("Left Rotation Position", rotateMotorLeft.getCurrentPosition());
        telemetry.addData("Right Rotation Position", rotateMotorRight.getCurrentPosition());
//        telemetry.addData("Left Axle Position", leftAxleServo.getPosition());
//        telemetry.addData("Right Axle Position", rightAxleServo.getPosition());
//        telemetry.addData("Left Gecko Position", leftGeckoServo.getPosition());
//        telemetry.addData("Right Gecko Position", rightGeckoServo.getPosition());
//        telemetry.addData("Left Slide Current", armMotorLeft.getCurrent());
//        telemetry.addData("Right Slide Current", armMotorRight.getCurrent());
//        telemetry.addData("Left Rotation Current", rotateMotorLeft.getCurrent());
//        telemetry.addData("Right Rotation Current", rotateMotorRight.getCurrent());

        if (isOverCurrentProtected) {
            telemetry.addLine("⚠️ OVERCURRENT PROTECTION ACTIVE ⚠️");
        }

        telemetry.update();
    }


}
