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

@TeleOp(name="Probox: Controls", group="Linear Opmode")
public class Probox_Controls extends LinearOpMode
{

    private Probox_Config config = new Probox_Config();

    @Override
    public void runOpMode()
    {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        config.HardwareMapAll(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {

            Movement_Controls();
            Lift_Contols();

        }

    }

    public void Movement_Controls()
    {

        config.BackwardPower  = gamepad1.left_trigger*2;
        config.ForwardPower = -gamepad1.right_trigger*2;
        config.FishTailPower = -gamepad1.right_stick_x*2;

        if(gamepad1.dpad_left == true)
        {

            config.LeftPower = 0.33;
            config.RightPower = -0.4;
            config.MiddlePower = -0.67;

            config.LeftWheel.setPower(config.LeftPower);
            config.RightWheel.setPower(config.RightPower);
            config.FishTail.setPower(config.MiddlePower);

        }

        if(gamepad1.dpad_right == true)
        {

            config.LeftPower = -0.33;
            config.RightPower = 0.4;
            config.MiddlePower = 0.67;

            config.LeftWheel.setPower(config.LeftPower);
            config.RightWheel.setPower(config.RightPower);
            config.FishTail.setPower(config.MiddlePower);

        }

        if(gamepad1.left_stick_x > 0)
        {
            config.LeftWheel.setPower(-1);
            config.RightWheel.setPower(1);
            config.FishTail.setPower(-1);
        }

        else if(gamepad1.left_stick_x < 0)
        {

            config.LeftWheel.setPower(1);
            config.RightWheel.setPower(-1);
            config.FishTail.setPower(1);

        }

        config.LeftWheel.setPower(config.ForwardPower);
        config.RightWheel.setPower(config.ForwardPower);
        config.LeftWheel.setPower(config.BackwardPower);
        config.RightWheel.setPower(config.BackwardPower);
        config.FishTail.setPower(config.FishTailPower);

    }

    public void Lift_Contols()
    {

        if(config.LiftMotor.getCurrentPosition() <= 3500)
        {
            if(gamepad2.x == true)
            {
                config.LiftMotor.setPower(-0.5);
            }
        }

        else if(config.LiftMotor.getCurrentPosition() >= 1000)
        {

            if(gamepad2.y == true)
            {
                config.LiftMotor.setPower(0.5);
            }

        }

        else
        {
            config.LiftMotor.setPower(0);
        }


        if(config.ExtendMotor.getCurrentPosition() >= 500)
        {

            if(gamepad2.dpad_up == true)
            {

                config.ExtendMotor.setPower(0.5);

            }

        }

        else if(config.ExtendMotor.getCurrentPosition() <= 3500)
        {

            if(gamepad2.dpad_down == true)
            {

                config.ExtendMotor.setPower(-0.5);

            }

        }

        else
        {

            config.ExtendMotor.setPower(0);

        }

        if (gamepad2.a == true)
        {

            if (config.Position_Clamp <= config.MAX_POS_Clamp)
            {
                config.Position_Clamp += config.INCREMENT ;
            }

        }

        else if(gamepad2.b == true)
        {

            if (config.Position_Clamp >= config.MIN_POS_Clamp)
            {
                config.Position_Clamp -= config.INCREMENT;
            }

        }

        if(gamepad2.left_bumper == true)
        {

            if(config.Position_Move >= config.MIN_POS_Move)
            {
                config.Position_Move -= config.INCREMENT;
            }

        }

        else if(gamepad2.right_bumper == true)
        {

            if(config.Position_Move <= config.MAX_POS_Move)
            {
                config.Position_Move += config.INCREMENT;
            }

        }

        config.Move_Servo_Pos =  config.MAX_POS_Move - config.Position_Move;

        config.Clamp_Servo.setPosition(config.Position_Clamp);
        config.Move_Servo_1.setPosition(config.Position_Move);
        config.Move_Servo_2.setPosition(config.Move_Servo_Pos);

    }

}