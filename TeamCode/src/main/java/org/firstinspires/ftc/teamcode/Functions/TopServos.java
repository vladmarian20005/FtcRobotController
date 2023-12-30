package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class TopServos {

    private Servo topServo;
    private boolean status1;
    public TopServos(Servo _TS)
    {
        topServo = _TS;
        status1 = false;
    }

    public void Open()
    {
        topServo.setPosition(1);
        status1 = true;
    }
    public void Close() {
        topServo.setPosition(0);
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
