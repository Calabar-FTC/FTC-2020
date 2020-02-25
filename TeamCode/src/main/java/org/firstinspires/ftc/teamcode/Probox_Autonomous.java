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

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Probox: Autonomous", group="Auto")
public class Probox_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private Probox_Config config = new Probox_Config();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double Counts_Per_Rev = 1440;    // eg: TETRIX Motor Encoder
    static final double Wheel_Diameter = 10.16;     // to get the circumfrence of the wheel
    static final double Speed = 1;
    static final double distance_per_rev = Math.PI * Wheel_Diameter;

    @Override
    public void runOpMode() {
        config.HardwareMapAll(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                config.LeftWheel.getCurrentPosition(),
                config.RightWheel.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        auto_comp(); //made to modularise steps for autonomous

//        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void move_a(double speed, double distance, double timeoutS)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            nuetral();

            // Determine new target position, and pass to motor controller
            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Left_wheel_pos = config.LeftWheel.getCurrentPosition();
            int Right_wheel_pos = config.RightWheel.getCurrentPosition();

            config.LeftWheel.setTargetPosition(Left_wheel_pos+distance_travel);
            config.RightWheel.setTargetPosition(Right_wheel_pos+distance_travel);

            // Turn On RUN_TO_POSITION
            config.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            config.LeftWheel.setPower(speed);
            config.RightWheel.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (config.LeftWheel.isBusy() && config.RightWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("","Left Wheel Position:%d\nRight Wheel Position: %d", config.LeftWheel.getCurrentPosition(), config.RightWheel.getCurrentPosition());

                telemetry.update();
            }

            brake();
        }
    }

    public void move_b(double speed, double distance, double timeoutS)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            nuetral();

            // Determine new target position, and pass to motor controller
            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Fish_Tail_pos = config.FishTail.getCurrentPosition();
            int Right_wheel_pos = config.RightWheel.getCurrentPosition();

            config.FishTail.setTargetPosition(Fish_Tail_pos+distance_travel);
            config.RightWheel.setTargetPosition(Right_wheel_pos+distance_travel);

            // Turn On RUN_TO_POSITION
            config.FishTail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            config.FishTail.setPower(speed);
            config.RightWheel.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (config.FishTail.isBusy() && config.RightWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Wheels","FishTail Position: %d\nRight Wheel Position: %d", config.FishTail.getCurrentPosition(), config.RightWheel.getCurrentPosition());
                telemetry.update();
            }

          brake();
        }
    }

    public void move_c(double speed, double distance, double timeoutS)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            nuetral();

            // Determine new target position, and pass to motor controller
            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Left_wheel_pos = config.LeftWheel.getCurrentPosition();
            int Fish_Tail_pos = config.FishTail.getCurrentPosition();

            config.LeftWheel.setTargetPosition(Left_wheel_pos+distance_travel);
            config.FishTail.setTargetPosition(Fish_Tail_pos-distance_travel);

            // Turn On RUN_TO_POSITION
            config.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.FishTail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            config.LeftWheel.setPower(speed);
            config.FishTail.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (config.LeftWheel.isBusy() && config.FishTail.isBusy()))
            {

                // Display it for the driver.
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
            config.RightWheel.setPower(config.RightPower);
            config.FishTail.setPower(config.FishTailPower);

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                // Display it for the driver.

                if (!config.FishTail.isBusy()) {
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
        config.LeftWheel.setPower(0);
        config.RightWheel.setPower(0);
        config.FishTail.setPower(0);

        config.LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        config.RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        config.FishTail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        config.LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.FishTail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        config.RightWheel.setTargetPosition(Right_wheel_pos-distance_travel);
        config.FishTail.setTargetPosition(Fish_Tail_pos+distance_travel);

        // Turn On RUN_TO_POSITION
        config.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        config.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        config.FishTail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        config.LeftWheel.setPower(config.LeftPower);
        config.RightWheel.setPower(config.RightPower);
        config.FishTail.setPower(config.FishTailPower);

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (config.LeftWheel.isBusy() && config.RightWheel.isBusy() && config.FishTail.isBusy())) {

            // Display it for the driver.
            telemetry.addData("","Left Wheel Position:%d\nRight Wheel Position: %d", config.LeftWheel.getCurrentPosition(), config.RightWheel.getCurrentPosition());

            telemetry.update();
        }

        brake();
    }

    public void grip_fundation(double position)
    {
        config.Move_Servo_1.setPosition(position);
        config.Move_Servo_2.setPosition(1-position);
    }

    public void auto_comp()
    {
        grip_block(1);
        sleep(100);
        move_lift(Speed,3,10);
        sleep(10);
        lateral_b(60, 10);
        sleep(10);
        move_a(Speed, 120, 10);
        sleep(10);
        lateral_b(60,10);
        sleep(10);
        grip_block(0);// start of dummy code
        sleep(10);
        move_a(Speed,-10,10);
        sleep(10);
        move_lift(Speed,-2,10);
        sleep(10);
        turn(0.5, 30, 10);
        sleep(10);
        move_b(Speed,10,10);
        sleep(10);
        grip_fundation(1);
        sleep(10);
        move_a(Speed,60,10);
        sleep(10);
        move_c(Speed,-60,10);
        sleep(10);
        grip_fundation(0);
        sleep(10);
        move_b(Speed, -105, 10);//end of dummy code
    }

    public void grip_block(double position)
    {
        config.Clamp_Servo.setPosition(position);
        config.Clamp_Servo_2.setPosition(1-position);
    }

    public void move_lift(double speed, double distance, double timeoutS)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            config.ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Determine new target position, and pass to motor controller
            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Extend_Motor_pos = config.ExtendMotor.getCurrentPosition();

            config.ExtendMotor.setTargetPosition(Extend_Motor_pos+distance_travel);

            // Turn On RUN_TO_POSITION
            config.ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            config.ExtendMotor.setPower(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && config.ExtendMotor.isBusy())
            {

                // Display it for the driver.
                telemetry.addData("","Lift Position:%d", config.ExtendMotor.getCurrentPosition());

                telemetry.update();
            }

            config.ExtendMotor.setPower(0);

            config.ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            config.ExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

