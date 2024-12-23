package org.firstinspires.ftc.teamcode.config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotConfig {
    public static class HardwareConfig {
        // Motors
        public static final boolean ENABLE_FL = true;
        public static final boolean ENABLE_FR = true;
        public static final boolean ENABLE_BL = true;
        public static final boolean ENABLE_BR = true;
        public static final boolean ENABLE_SLIDE_LEFT = true;
        public static final boolean ENABLE_SLIDE_RIGHT = true;
        public static final boolean ENABLE_ROTATE_LEFT = true;
        public static final boolean ENABLE_ROTATE_RIGHT = true;

        // Servos
        public static final boolean ENABLE_LEFT_AXLE = false;
        public static final boolean ENABLE_RIGHT_AXLE = false;
        public static final boolean ENABLE_LEFT_GECKO = false;
        public static final boolean ENABLE_RIGHT_GECKO = false;
    }

    private HardwareMap hardwareMap;

    public RobotConfig(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public DcMotor getMotorIfEnabled(String name, boolean isEnabled) {
        return isEnabled ? hardwareMap.dcMotor.get(name) : null;
    }

    public DcMotorEx getMotorExIfEnabled(String name, boolean isEnabled) {
        return isEnabled ? hardwareMap.get(DcMotorEx.class, name) : null;
    }

    public Servo getServoIfEnabled(String name, boolean isEnabled) {
        return isEnabled ? hardwareMap.servo.get(name) : null;
    }
}