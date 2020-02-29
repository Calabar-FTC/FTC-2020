package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    public Servo Clamp_Servo = null;
    public Servo Clamp_Servo_2 = null;
    public Servo Move_Servo_1 = null; // move servos are used to clamp the foundation
    public Servo Move_Servo_2 = null;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    public HardwareMap mapping; // the mapping object for the devices

    public double ForwardPower = 0, BackwardPower = 0, FishTailPower = 0, LeftPower = 0, RightPower = 0;

    public int lift_min_position = 0;
    public int extend_min_position = 0;
    public int lift_mid_position = 0;
    public int extend_max_position = 0;
    public int greenColorStop = 350;
    public int blueColorStop = 200;
    public double min_colo_distance = 6;



    public void HardwareMapAll(HardwareMap mapping)
    {
        this.mapping = mapping;
        LeftWheel  = this.mapping.get(DcMotor.class, "LeftWheel");
        RightWheel = this.mapping.get(DcMotor.class, "RightWheel");
        FishTail = this.mapping.get(DcMotor.class, "FishTail");
        LiftMotor = this.mapping.get(DcMotor.class, "LiftMotor");
        Clamp_Servo = this.mapping.get(Servo.class, "Clamp_Servo");
        Clamp_Servo_2 = this.mapping.get(Servo.class, "Clamp_Servo_2");
        Move_Servo_1 = this.mapping.get(Servo.class, "Move_Servo_1");
        Move_Servo_2 = this.mapping.get(Servo.class, "Move_Servo_2");
        ExtendMotor = this.mapping.get(DcMotor.class, "ExtendMotor");
        colorSensor = this.mapping.get(ColorSensor.class, "color_distance");
        distanceSensor = this.mapping.get(DistanceSensor.class, "color_distance");

        // Set the direction of the motors and servo
        LeftWheel.setDirection(DcMotor.Direction.FORWARD);
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        FishTail.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ExtendMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        Clamp_Servo.setDirection(Servo.Direction.FORWARD);
        Clamp_Servo_2.setDirection(Servo.Direction.FORWARD);
        Move_Servo_1.setDirection(Servo.Direction.FORWARD);
        Move_Servo_2.setDirection(Servo.Direction.FORWARD);

        //Reset the encoders for all the motors and set them to run in encoder mode
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FishTail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FishTail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the maximum and minimum values for the lift system motors
        lift_min_position = LiftMotor.getCurrentPosition();
        extend_min_position = ExtendMotor.getCurrentPosition();
        lift_mid_position = (int) ((LiftMotor.getCurrentPosition() + 5100)*0.75);
        extend_max_position = -204;
        greenColorStop = colorSensor.green()+150;
        blueColorStop = colorSensor.blue()+100;
        Clamp_Servo_2.setPosition(0);
        Clamp_Servo.setPosition(1);
    }
}
