package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;

@TeleOp
@Disabled
public class GamepadTeleOp extends OpMode {
    GamepadCalc gamepadCalc;
    double movement;


    @Override
    public void init() {
        gamepadCalc = new GamepadCalc(this);
    }

    @Override
    public void loop() {
        gamepadCalc.calculate();
        movement = gamepadCalc.getGamepad1().left_trigger;
    }
}
