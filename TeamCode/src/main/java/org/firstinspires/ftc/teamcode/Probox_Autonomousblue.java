/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;

@Autonomous(name="Probox: Autonomous Blue 2", group="Auto")
public class Probox_Autonomousblue extends LinearOpMode
{
    private Probox_Config config = new Probox_Config();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double Counts_Per_Rev = 1440;
    static final double Wheel_Diameter = 10.16;
    static final double Speed = 1;
    static final double distance_per_rev = Math.PI * Wheel_Diameter;

    @Override
    public void runOpMode()
    {
        config.HardwareMapAll(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        telemetry.addData("Motors",  "Left Motor: %d\nRight Motor: %d",  config.LeftWheel.getCurrentPosition(), config.RightWheel.getCurrentPosition());
        telemetry.update();

        waitForStart();

        //auto_comp();
        simple();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void move_a(double speed, double distance, double timeoutS)
    {
        if (opModeIsActive())
        {
            nuetral();

            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Left_wheel_pos = config.LeftWheel.getCurrentPosition();
            int Right_wheel_pos = config.RightWheel.getCurrentPosition();

            config.LeftWheel.setTargetPosition(Left_wheel_pos-distance_travel);
            config.RightWheel.setTargetPosition(Right_wheel_pos+distance_travel);

            config.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            config.LeftWheel.setPower(speed);
            config.RightWheel.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS))
            {
                if(distance > 0)
                {
                    if(config.LeftWheel.getCurrentPosition() <= Left_wheel_pos-distance_travel)
                    {
                        brake();
                        break;
                    }
                }

                else if(distance < 0)
                {
                    if(config.LeftWheel.getCurrentPosition() >  Left_wheel_pos-distance_travel)
                    {
                        brake();
                        break;
                    }
                }
                telemetry.addData("Wheel Position","Left Wheel Position:%d\nRight Wheel Position: %d", config.LeftWheel.getCurrentPosition(), config.RightWheel.getCurrentPosition());
                telemetry.update();
            }
            brake();
        }
    }

    public void move_b(double speed, double distance, double timeoutS)
    {
        if (opModeIsActive())
        {
            nuetral();

            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Fish_Tail_pos = config.FishTail.getCurrentPosition();
            int Right_wheel_pos = config.RightWheel.getCurrentPosition();

            config.FishTail.setTargetPosition(Fish_Tail_pos+distance_travel);
            config.RightWheel.setTargetPosition(Right_wheel_pos+distance_travel);

            config.FishTail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            config.FishTail.setPower(speed);
            config.RightWheel.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (config.FishTail.isBusy() && config.RightWheel.isBusy()))
            {
                telemetry.addData("Wheels","FishTail Position: %d\nRight Wheel Position: %d", config.FishTail.getCurrentPosition(), config.RightWheel.getCurrentPosition());
                telemetry.update();
            }
          brake();
        }
    }

    public void move_c(double speed, double distance, double timeoutS)
    {
        if (opModeIsActive())
        {
            nuetral();

            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Left_wheel_pos = config.LeftWheel.getCurrentPosition();
            int Fish_Tail_pos = config.FishTail.getCurrentPosition();

            config.LeftWheel.setTargetPosition(Left_wheel_pos-distance_travel);
            config.FishTail.setTargetPosition(Fish_Tail_pos+distance_travel);

            config.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.FishTail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            config.LeftWheel.setPower(speed);
            config.FishTail.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (config.LeftWheel.isBusy() && config.FishTail.isBusy()))
            {
                telemetry.addData("Wheels","Left Wheel Position:%d\nFishTail Position: %d", config.LeftWheel.getCurrentPosition(), config.FishTail.getCurrentPosition());
                telemetry.update();
            }

            brake();
        }
    }

    public void lateral_b(double distance, double timeoutS)
    {
        if (opModeIsActive())
        {
            nuetral();

            int distance_travel = (int) (distance / distance_per_rev * Counts_Per_Rev);
            int Fish_Tail_pos = config.FishTail.getCurrentPosition();

            config.LeftPower = -0.33;
            config.RightPower = 0.4;
            config.FishTailPower = 0.67;

            config.FishTail.setTargetPosition(Fish_Tail_pos + distance_travel);

            config.FishTail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            config.LeftWheel.setPower(config.LeftPower);
            config.RightWheel.setPower(1*-config.RightPower);
            config.FishTail.setPower(config.FishTailPower);


            while (opModeIsActive() && (runtime.seconds() < timeoutS))
            {
                if (!config.FishTail.isBusy())
                {
                    config.LeftWheel.setPower(0);
                    config.RightWheel.setPower(0);
                    config.FishTail.setPower(0);
                    brake();
                    break;
                }
                telemetry.addData("FishTail Count:", config.FishTail.getCurrentPosition());
                telemetry.update();
            }
            brake();
        }
    }

    public void nuetral()
    {
        config.LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        config.RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        config.FishTail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void brake()
    {
        config.RightWheel.setPower(0);
        config.FishTail.setPower(0);
        config.LeftWheel.setPower(0);

        config.RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        config.FishTail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        config.LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        config.RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.FishTail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turn(double speed, double distance, double timeoutS)
    {
        nuetral();

        config.LeftPower = 1*speed;
        config.RightPower = 1*speed;
        config.FishTailPower = 1*speed;

        int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
        int Left_wheel_pos = config.LeftWheel.getCurrentPosition();
        int Right_wheel_pos = config.RightWheel.getCurrentPosition();
        int Fish_Tail_pos = config.RightWheel.getCurrentPosition();

        config.LeftWheel.setTargetPosition(Left_wheel_pos+distance_travel);
        config.RightWheel.setTargetPosition(Right_wheel_pos+distance_travel);
        config.FishTail.setTargetPosition(Fish_Tail_pos+distance_travel);

        config.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        config.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        config.FishTail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        config.LeftWheel.setPower(config.LeftPower);
        config.RightWheel.setPower(config.RightPower);
        config.FishTail.setPower(config.FishTailPower);

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (config.LeftWheel.isBusy() && config.RightWheel.isBusy() && config.FishTail.isBusy()))
        {
            if(config.LeftWheel.getCurrentPosition() <= Left_wheel_pos-distance_travel)
            {
                brake();
                break;
            }

            telemetry.addData("","Left Wheel Position:%d\nRight Wheel Position: %d", config.LeftWheel.getCurrentPosition(), config.RightWheel.getCurrentPosition());
            telemetry.update();
        }
        brake();
    }

    public void auto_comp()
    {
        grip_block(1);
       move_a(Speed, 89, 10);
       grip_block(0);
       move_a(Speed, -30, 10);
       turn(0.5, 30, 10);
       move_a(Speed, 146, 10);
       move_lift(0.5, -0.2, 10);
       lateral_b(15, 10);
       move_a(0.5, 15, 10);
       grip_block(1);
       move_lift(0.5, 0.2, 10);
       turn(Speed, 15, 10);
       move_a(Speed, 60, 10);
       move_lift(0.5, -0.2, 10);
       move_c(Speed, 60, 10);
    }

    public void grip_block(double position)
    {
        config.Clamp_Servo.setPosition(position);
        config.Clamp_Servo_2.setPosition(1-position);
        sleep(1000);
    }

    public void move_lift(double speed, double distance, double timeoutS)
    {
        if (opModeIsActive())
        {
            config.LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Lift_Motor_Pos = config.LiftMotor.getCurrentPosition();

            config.LiftMotor.setTargetPosition(Lift_Motor_Pos+distance_travel);

            config.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            config.LiftMotor.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && config.LiftMotor.isBusy())
            {
                telemetry.addData("","Lift Position:%d", config.LiftMotor.getCurrentPosition());
                telemetry.update();
            }
            config.LiftMotor.setPower(0);
            config.LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            config.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void simple()
    {
        move_a(Speed, 80, 10);
    }
}