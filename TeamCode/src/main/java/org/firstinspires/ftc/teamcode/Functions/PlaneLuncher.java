package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLuncher {

    private Servo servo;
    public PlaneLuncher(Servo _LS)
    {
        servo = _LS;
    }

    public void Open()
    {
        servo.setPosition(0.5);
    }
    public void Close() {
        servo.setPosition(1);
    }


}
