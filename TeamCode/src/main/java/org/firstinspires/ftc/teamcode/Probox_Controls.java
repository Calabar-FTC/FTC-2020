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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="Probox: Controls", group="FTC 2020")
public class Probox_Controls extends LinearOpMode
{
    // robot configuration object used to store all the sensors configuration
    private Probox_Config config = new Probox_Config();

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        config.HardwareMapAll(hardwareMap);
        telemetry.addData("Configuration", "Completed");
        telemetry.update();

        config.Move_Servo_1.setPosition(0);
        config.Move_Servo_2.setPosition(1);

        waitForStart();

        config.runtime.reset();
        while (opModeIsActive())
        {
            // robot chassis movement module
            movement_Controls();
            lift_Contols();
            clamp_Control();
            foundation_gripper_control();
            telemetry.addData("Status", "Run Time: " + config.runtime.toString());
            telemetry.update();
        }
    }

    public void movement_Controls()
    {
        // get values from the controller
        config.BackwardPower  = Range.clip(gamepad1.right_trigger*2,-1.0,1.0);
        config.ForwardPower = Range.clip(-gamepad1.left_trigger*2,-1.0,1.0);
        config.FishTailPower = Range.clip(-gamepad1.right_stick_x*1.7,-1.0,1.0);

        //Lateral Movement to the left
        if(gamepad1.dpad_left)
        {
            config.LeftPower = 0.33;
            config.RightPower = -0.4;
            config.FishTailPower = -0.67;
        }
        //Lateral movement to the right
        else if(gamepad1.dpad_right)
        {
            config.LeftPower = -0.33;
            config.RightPower = 0.4;
            config.FishTailPower = 0.67;
        }
        //360 degree turn to the left
        else if(gamepad1.left_stick_x > 0)
        {
            config.LeftPower = -1*gamepad1.left_stick_x;
            config.RightPower = 1*gamepad1.left_stick_x;
            config.FishTailPower = -1*gamepad1.left_stick_x;
        }
        //360 degree turn to the right
        else if(gamepad1.left_stick_x < 0)
        {
            config.LeftPower = 1*-gamepad1.left_stick_x;
            config.RightPower = -1*-gamepad1.left_stick_x;
            config.FishTailPower = 1*-gamepad1.left_stick_x;
        }
        else
        {
            if(config.BackwardPower > 0)
            {
                config.LeftPower = config.BackwardPower;
                config.RightPower = config.BackwardPower;
            }
            else if(config.ForwardPower < 0)
            {
                config.LeftPower = config.ForwardPower;
                config.RightPower = config.ForwardPower;
            }
            else
            {
                config.LeftPower = 0;
                config.RightPower = 0;
            }
        }

        config.LeftWheel.setPower(config.LeftPower);
        config.RightWheel.setPower(config.RightPower);
        config.FishTail.setPower(config.FishTailPower);
        telemetry.addData("Wheels", "right_Wheel: %.2f | left_Wheel: %.2f | fishTail: %.2f",
                config.RightPower, config.LeftPower, config.FishTailPower);
    }

    public void lift_Contols()
    {
        // lift integration
        if(gamepad2.y)
        {
            if((config.LiftMotor.getCurrentPosition() < config.lift_mid_position) || (config.colorSensor.green() < config.greenColorStop && config.colorSensor.blue() < config.blueColorStop&& config.distanceSensor.getDistance(DistanceUnit.CM) > config.min_colo_distance))
            {
                config.LiftMotor.setPower(0.5);
            }
            else if(config.ExtendMotor.getCurrentPosition() < config.extend_max_position)
            {
                config.ExtendMotor.setPower(0.5);
            }
        }
        else if(gamepad2.dpad_right)
        {
            if(config.ExtendMotor.getCurrentPosition() > config.extend_min_position)
            {
                config.ExtendMotor.setPower(-0.5);
            }
            else if(config.LiftMotor.getCurrentPosition() > config.lift_min_position)
            {
            config.LiftMotor.setPower(-0.5);
            }
        }
        else if(gamepad2.dpad_down)
        {
            config.ExtendMotor.setPower(-0.5);
        }
        else if(gamepad2.dpad_up)
        {
            config.ExtendMotor.setPower(0.5);
        }
        else
        {
            config.LiftMotor.setPower(0);
            config.ExtendMotor.setPower(0);
        }
        telemetry.addData("Blue ", config.colorSensor.blue());
        telemetry.addData("color and distance","color green: %d | distance: %s",config.colorSensor.green(),String.format(Locale.US, "%.02f", config.distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Lift Mechanism","lift1: %d | lift2: %d", config.LiftMotor.getCurrentPosition(), config.ExtendMotor.getCurrentPosition());
    }

    public void clamp_Control()
    {
        if(gamepad2.a)
        {
            config.Clamp_Servo.setPosition(1);
            config.Clamp_Servo_2.setPosition(0);
        }

        else if(gamepad2.b)
        {
            config.Clamp_Servo.setPosition(0);
            config.Clamp_Servo_2.setPosition(1);
        }

        else if(gamepad2.x)
        {
            config.Clamp_Servo.setPosition(0.2);
            config.Clamp_Servo_2.setPosition(0.8);
        }

        telemetry.addData("Clamp","Clamp 1 position: %.2f\nClamp 2 position: %.2f",config.Clamp_Servo.getPosition(), config.Clamp_Servo_2.getPosition());
    }

    private void foundation_gripper_control()
    {
        // slowly lowers or lift the grippers when the button is pressed
        if(gamepad2.left_bumper)
        {
            config.Move_Servo_1.setPosition(0);
            config.Move_Servo_2.setPosition(1);
        }

        else if(gamepad2.right_bumper)
        {
            config.Move_Servo_1.setPosition(1);
            config.Move_Servo_2.setPosition(0);
        }

        telemetry.addData("Grippers"," grip_1: %.2f | grip_2: %.2f",config.Move_Servo_1.getPosition(), config.Move_Servo_2.getPosition());

    }

}