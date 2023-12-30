package org.firstinspires.ftc.teamcode.Functions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServos {

    private Servo servo;
    private boolean status1;
    public ClawServos(Servo _LS)
    {
        servo = _LS;
        status1 = false;
    }

    public void Open()
    {
        servo.setPosition(1);
        status1 = true;
    }
    public void Close() {
        servo.setPosition(0);
        status1 = false;
    }

    public void Switch(){
        if(status1){
            Close();
        }
        else{
            Open();
        }
    }
    double currentWaitTime =0;
    double currentTimeStamp =0;
    public void SwitchAndWait(double x, double currentRuntime){

        if(currentWaitTime ==0|| currentTimeStamp + currentWaitTime <=currentRuntime){
            Switch();
            currentTimeStamp =currentRuntime;
            currentWaitTime =x;
        }
    }

}
