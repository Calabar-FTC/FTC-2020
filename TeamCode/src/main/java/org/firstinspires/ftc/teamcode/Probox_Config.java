package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Probox_Config
{
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor LeftWheel = null;
    public DcMotor RightWheel = null;
    public DcMotor FishTail = null;
    public DcMotor LiftMotor = null;
    public DcMotor ExtendMotor = null;
    public HardwareMap mapping;

    public final double INCREMENT = 0.01;
    public final double MAX_POS_Clamp =  1.0, MAX_POS_Move = 1.0;
    public final double MIN_POS_Clamp =  0.0, MIN_POS_Move = 0.0;

    public Servo Clamp_Servo, Move_Servo_1, Move_Servo_2;
    double  Position_Clamp = 1, Position_Move = 1, Move_Servo_Pos = 1;
    double ForwardPower = 0, BackwardPower = 0, FishTailPower = 0, LeftPower = 0, RightPower = 0, MiddlePower = 0;

    public void HardwareMapAll(HardwareMap mapping)
    {
        LeftWheel.setDirection(DcMotor.Direction.FORWARD);
        RightWheel.setDirection(DcMotor.Direction.REVERSE);

        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.mapping = mapping;
        LeftWheel  = this.mapping.get(DcMotor.class, "LeftWheel");
        RightWheel = this.mapping.get(DcMotor.class, "RightWheel");
        FishTail = this.mapping.get(DcMotor.class, "FishTail");
        LiftMotor = this.mapping.get(DcMotor.class, "LiftMotor");
        Clamp_Servo = this.mapping.get(Servo.class, "Clamp_Servo");
        Move_Servo_1 = this.mapping.get(Servo.class, "Move_Servo_1");
        Move_Servo_2 = this.mapping.get(Servo.class, "Move_Servo_2");
        ExtendMotor = this.mapping.get(DcMotor.class, "ExtendMotor");
    }
}
