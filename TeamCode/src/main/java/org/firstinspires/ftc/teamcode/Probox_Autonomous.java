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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@Autonomous(name="Probox: Autonomous", group="Pushbot")
public class Probox_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private Probox_Config config = new Probox_Config();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double Counts_Per_Rev = 1440;    // eg: TETRIX Motor Encoder
    static final double Wheel_Diameter = 10.16;     // to get the circumfrence of the wheel
    static final double Speed = 0.6;
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

        encoderDrive(Speed,100,5);  // S1: Forward 47 Inches with 5 Sec timeout
        config.Clamp_Servo.setPosition(1);


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDrive(double speed, double distance, double timeoutS)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int distance_travel = (int) (distance/distance_per_rev * Counts_Per_Rev );
            int Left_wheel_pos= config.LeftWheel.getCurrentPosition();
            int Right_wheel_pos= config.RightWheel.getCurrentPosition();

            config.LeftWheel.setTargetPosition(Left_wheel_pos+distance_travel);
            config.RightWheel.setTargetPosition(Right_wheel_pos+distance_travel);

            // Turn On RUN_TO_POSITION
            config.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            config.LeftWheel.setTargetPosition(Left_wheel_pos+distance_travel);
            config.RightWheel.setTargetPosition(Right_wheel_pos+distance_travel);

            // reset the timeout time and start motion.
            runtime.reset();
            config.LeftWheel.setPower(Math.abs(speed));
            config.RightWheel.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (config.LeftWheel.isBusy() && config.RightWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to 100\n");
                telemetry.addData("Path2",  "Current Position: %7d",
                                            config.LeftWheel.getCurrentPosition(),
                                            config.RightWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            config.LeftWheel.setPower(0);
            config.RightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            config.LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            config.RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}
