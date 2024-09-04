package org.firstinspires.ftc.teamcode.Orig.TeleOp.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ClawArmEncoder {
    private final DcMotor motor;
    private int motorPos;
    double integralSum = 0;
    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    public void Init()
    {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motor.setDirection(DcMotor.Direction.REVERSE);

        motorPos = 0 ;
    }

    public ClawArmEncoder(DcMotor motor) {
        this.motor = motor;
        Init();
    }

    public double PIDControl(double reference, double state)
    {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki) + (reference * kf);
        return output;

    }

    public void goTo(int motorTarget)
    {
        motorPos = 0;

        motorPos+=motorTarget;

        motor.setTargetPosition(motorPos);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = PIDControl(motorTarget, motor.getCurrentPosition());

        motor.setPower(1);
    }

}
