/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.TeleOpV3;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The main teleop program which includes drivetrain and firetruck operations. The telemetry at the
 * end gives the status of all encoded actuators.
 */

@TeleOp(name="Firetruck TeleOp Test", group="Robot")
@Disabled
public class FiretruckTeleOpTest extends LinearOpMode {

    RobotHardwareV3 robot = new RobotHardwareV3(this);

    @Override
    public void runOpMode() {
        double axial = 0;
        double lateral = 0;
        double yaw = 0;
        double drillBuffer = 0.5;

        // initialize all the hardware, using the hardware class
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode
            yaw = -gamepad1.left_stick_y * Constants.YAW_DAMP;  // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x * Constants.LATERAL_DAMP;
            axial = gamepad1.right_stick_x * Constants.AXIAL_DAMP;

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(axial, lateral, yaw);

            // Use gamepad left & right Triggers to hold open the erector or pincher servos respectively
            if (gamepad2.right_trigger >= 0.5) {
                robot.firetruck.setPincherPosition(Constants.PINCHER_POS);
            } else {
                robot.firetruck.setPincherToIdle();
            }

            if (gamepad2.left_trigger >= 0.5) {
                robot.firetruck.setErectorPosition(Constants.ERECTOR_POS_UP);
            } else {
                robot.firetruck.setErectorToIdle();
            }

            /*
            if(gamepad1.dpad_left) {
                drillBuffer += 0.01;
            } else if(gamepad1.dpad_right) {
                drillBuffer -= 0.01;
            }
            drillBuffer = Range.clip(drillBuffer,0.0,1.0);
            robot.firetruck.setDrillPosition(drillBuffer);
             */
            if(gamepad2.dpad_left) {
                robot.firetruck.driveDrill(-Constants.DRILL_SPEED);
            } else if(gamepad2.dpad_right) {
                robot.firetruck.driveDrill(Constants.DRILL_SPEED);
            } else {
                robot.firetruck.stopDrill();
            }

            // Use gamepad buttons to move lift up (Y) and down (A) and turntable left (X) and right (B)
            if (gamepad2.y)
                robot.firetruck.driveLift(Constants.LIFT_UP_POW);
            else if (gamepad2.a)
                robot.firetruck.driveLift(Constants.LIFT_DOWN_POW);
            else
                robot.firetruck.driveLift(0);

            if (gamepad2.x)
                robot.firetruck.driveTurntableLeft(Constants.TURNTABLE_POW);
            else if (gamepad2.b)
                robot.firetruck.driveTurntableRight(Constants.TURNTABLE_POW);
            else
                robot.firetruck.driveTurntable(0);

            // Use gamepad left & right Bumpers to tilt the firetruck up and down respectively.
            // If no input given, tilt moves to an idle position using RUN_TO_POSITION to the latest
            // driven position.
            if (gamepad2.left_bumper)
                robot.firetruck.driveTilt(Constants.TILT_UP_POW);
            else if (gamepad2.right_bumper)
                robot.firetruck.driveTilt(Constants.TILT_DOWN_POW);
            else
                robot.firetruck.tiltIdle(Constants.TILT_IDLE_POW);//robot.firetruck.driveTilt(0);

            // Manually resets the motor encoders such that the current position becomes the zero point
            if(gamepad2.dpad_up){
                robot.firetruck.tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.firetruck.tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");

            telemetry.addData("Drive Power", "%.2f", axial);
            telemetry.addData("Turn Power", "%.2f", yaw);
            telemetry.addData("Strafe Power", "%.2f", lateral);
            telemetry.addData("Tilt position", robot.firetruck.tilt.getCurrentPosition());
            telemetry.addData("Tilt target", robot.firetruck.tilt.getTargetPosition());
            telemetry.addData("Lift position", robot.firetruck.tilt.getCurrentPosition());
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            //sleep(50);
        }
    }
}
