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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="David Lynch: Fish Tail Design", group="Linear Opmode")
public class DavidLynchFishTailDesign extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftWheel = null;
    private DcMotor RightWheel = null;
    private DcMotor FishTail = null;
    private DcMotor LiftMotor = null;
    private DcMotor ExtendMotor = null;

    static final double INCREMENT = 0.01;
    static final double MAX_POS_Clamp =  1.0, MAX_POS_Move = 1.0;
    static final double MIN_POS_Clamp =  0.0, MIN_POS_Move = 0.0;

    private Servo Clamp_Servo, Move_Servo_1, Move_Servo_2;
    double  Position_Clamp = 1, Position_Move = 1, Move_Servo_Pos = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftWheel  = hardwareMap.get(DcMotor.class, "LeftWheel");
        RightWheel = hardwareMap.get(DcMotor.class, "RightWheel");
        FishTail = hardwareMap.get(DcMotor.class, "FishTail");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        Clamp_Servo = hardwareMap.get(Servo.class, "Clamp_Servo");
        Move_Servo_1 = hardwareMap.get(Servo.class, "Move_Servo_1");
        Move_Servo_2 = hardwareMap.get(Servo.class, "Move_Servo_2");
        ExtendMotor = hardwareMap.get(DcMotor.class, "ExtendMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftWheel.setDirection(DcMotor.Direction.FORWARD);
        RightWheel.setDirection(DcMotor.Direction.REVERSE);

        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

       // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double ForwardPower = 0, BackwardPower = 0, FishTailPower = 0, LeftPower = 0, RightPower = 0, MiddlePower = 0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
//            double drive = -gamepad1.left_stick_y;
//            double turn  =  gamepad1.right_stick_x;
//            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
             BackwardPower  = gamepad1.left_trigger*2;
             ForwardPower = -gamepad1.right_trigger*2;
             FishTailPower = -gamepad1.right_stick_x*2;

//             Send calculated power to wheels
            if(gamepad2.dpad_down == true)
            {
                ExtendMotor.setPower(-0.5);
            }

            else if(gamepad2.dpad_up == true)
            {
                ExtendMotor.setPower(0.5);
            }

            else
            {
                ExtendMotor.setPower(0);
            }

            if (gamepad2.a == true)
            {
                if (Position_Clamp <= MAX_POS_Clamp)
                {
                    Position_Clamp += INCREMENT ;
                }
            }

            else if(gamepad2.b == true)
            {
                if (Position_Clamp >= MIN_POS_Clamp)
                {
                    Position_Clamp -= INCREMENT;
                }
            }

            if(gamepad2.left_bumper == true)
            {
                if(Position_Move >= MIN_POS_Move)
                {
                    Position_Move -= INCREMENT;
                }
            }

            else if(gamepad2.right_bumper == true)
            {
                if(Position_Move <= MAX_POS_Move)
                {
                    Position_Move += INCREMENT;
                }
            }

            if(gamepad1.dpad_left == true)
            {
                LeftPower = 0.33;
                RightPower = -0.4;
                MiddlePower = -0.67;
            }

            if(gamepad1.dpad_right == true)
            {
                LeftPower = -0.33;
                RightPower = 0.4;
                MiddlePower = 0.67;
            }

            if(gamepad2.y == true)
            {
                LiftMotor.setPower(0.5);
            }

            else if(gamepad2.x == true)
            {
                LiftMotor.setPower(-0.5);
            }

            else
            {
                LiftMotor.setPower(0);
            }

            LeftWheel.setPower(LeftPower);
            RightWheel.setPower(RightPower);
            FishTail.setPower(MiddlePower);

            if(gamepad1.left_stick_x > 0)
            {
                LeftWheel.setPower(-1);
                RightWheel.setPower(1);
                FishTail.setPower(-1);
            }

            else if(gamepad1.left_stick_x < 0)
            {
                LeftWheel.setPower(1);
                RightWheel.setPower(-1);
                FishTail.setPower(1);
            }

            LeftWheel.setPower(ForwardPower);
            RightWheel.setPower(ForwardPower);
            LeftWheel.setPower(BackwardPower);
            RightWheel.setPower(BackwardPower);
            FishTail.setPower(FishTailPower);

            Move_Servo_Pos =  MAX_POS_Move - Position_Move;

            Clamp_Servo.setPosition(Position_Clamp);
            Move_Servo_1.setPosition(Position_Move);
            Move_Servo_2.setPosition(Move_Servo_Pos);
            //sleep(CYCLE_MS);
            //idle();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "ForwardPower: (%.2f)\nBackwardPower: (%.2f)\nFishTailPower: (%.2f)\nDpadLeft: %b\nDpadRight: %b", ForwardPower, BackwardPower, FishTailPower, gamepad1.dpad_left, gamepad1.dpad_right);
            telemetry.addData("Motors", "Lift Motor:\t%d\nExtend Motor:\t%d\n", LiftMotor.getCurrentPosition(), ExtendMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}