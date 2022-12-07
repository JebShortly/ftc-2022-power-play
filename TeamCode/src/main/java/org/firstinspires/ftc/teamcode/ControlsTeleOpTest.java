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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The main teleop program which includes drivetrain and firetruck operations. The telemetry at the
 * end gives the status of all encoded actuators.
 */

@TeleOp(name="TeleOp", group="Robot")
//@Disabled
public class ControlsTeleOpTest extends LinearOpMode {

    RobotHardwareV4 robot = new RobotHardwareV4(this);

    public enum ErectState {
        BUTTON_START,
        BUTTON_PRESSED_Y,
        BUTTON_PRESSED_A,
        BUTTON_RELEASED_Y,
        BUTTON_RELEASED_A,
        BUTTON_END
    }
    ErectState erectState = ErectState.BUTTON_START;
    ElapsedTime erectTimer = new ElapsedTime();

    public enum PinchState {
        BUTTON_START,
        BUTTON_PRESSED,
        BUTTON_RELEASED,
        BUTTON_END
    }
    PinchState pinchState = PinchState.BUTTON_START;
    ElapsedTime pinchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        double axial = 0;
        double lateral = 0;
        double yaw = 0;
        double erectorAcc = 0;

        // initialize all the hardware, using the hardware class
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode
            axial = -gamepad1.left_stick_y * ConstantsV4.YAW_DAMP;  // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x * ConstantsV4.LATERAL_DAMP;
            yaw = gamepad1.right_stick_x * ConstantsV4.AXIAL_DAMP;

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(axial, lateral, yaw);

            robot.firetruck.driveTurntable(gamepad2.left_stick_x * ConstantsV4.TURNTABLE_POW);
            if(gamepad2.left_stick_y < 0){
                robot.firetruck.driveLift(-gamepad2.left_stick_y * ConstantsV4.LIFT_UP_POW);
            } else if(gamepad2.left_stick_y > 0){
                robot.firetruck.driveLift(-gamepad2.left_stick_y * -ConstantsV4.LIFT_DOWN_POW);
            } else {
                //robot.firetruck.driveLift(0);
                robot.firetruck.liftIdle(ConstantsV4.LIFT_IDLE_POW);
            }
            if(gamepad2.right_stick_y < 0){
                robot.firetruck.driveTilt(-gamepad2.right_stick_y * -ConstantsV4.TILT_DOWN_POW);
            } else if(gamepad2.right_stick_y > 0){
                robot.firetruck.driveTilt(-gamepad2.right_stick_y * ConstantsV4.TILT_UP_POW);
            } else {
                robot.firetruck.tiltIdle(ConstantsV4.TILT_IDLE_POW);
            }
            /*
            if(gamepad1.y) {
                erectorAcc += 0.01;
            } else if(gamepad1.a) {
                erectorAcc -= 0.01;
            }
            erectorAcc = Range.clip(erectorAcc+ConstantsV4.ERECTOR_IDLE,0.0,1.0);
            robot.firetruck.setErectorPosition(erectorAcc);
            */
            /*
            if(gamepad2.y) {
                robot.firetruck.driveErector(-ConstantsV4.ERECTOR_SPEED);
            } else if(gamepad2.a) {
                robot.firetruck.driveErector(ConstantsV4.ERECTOR_SPEED);
            } else {
                robot.firetruck.stopErector();
            }
            */
            /*
            if(gamepad2.y) {
                robot.firetruck.setErectorToIdle();
            } else if(gamepad2.a) {
                robot.firetruck.setErectorPosition(ConstantsV4.ERECTOR_POS_DOWN);
            }
             */
            /*
            switch (erectState){
                case BUTTON_START:
                    if(gamepad2.y){
                        robot.firetruck.setErectorPosition(ConstantsV4.ERECTOR_POS_UP);
                        pinchTimer.reset();
                        erectState = ErectState.BUTTON_RELEASED_Y;
                    }
                    if(gamepad2.a){
                        robot.firetruck.setErectorPosition(ConstantsV4.ERECTOR_POS_DOWN);
                        pinchTimer.reset();
                        erectState = ErectState.BUTTON_RELEASED_A;
                    }
                    break;
                case BUTTON_RELEASED_Y:
                    if(gamepad2.a && pinchTimer.milliseconds() >= 300){
                        robot.firetruck.setErectorToIdle();
                        erectState = ErectState.BUTTON_END;
                    }
                    break;
                case BUTTON_RELEASED_A:
                    if(gamepad2.y && pinchTimer.milliseconds() >= 300){
                        robot.firetruck.setErectorToIdle();
                        erectState = ErectState.BUTTON_END;
                    }
                    break;
                case BUTTON_END:
                    if(!(gamepad2.a || gamepad2.y)){
                        erectState = ErectState.BUTTON_START;
                    }
                    break;
                default:
                    erectState = ErectState.BUTTON_START;
            }
            */
            switch (pinchState){
                case BUTTON_START:
                    if(gamepad2.right_bumper){
                        robot.firetruck.setPincherPosition(ConstantsV4.PINCHER_POS);
                        pinchTimer.reset();
                        pinchState = PinchState.BUTTON_PRESSED;
                    }
                    break;
                case BUTTON_PRESSED:
                    if(!gamepad2.right_bumper){
                        pinchState = PinchState.BUTTON_RELEASED;
                    }
                    break;
                case BUTTON_RELEASED:
                    if(gamepad2.right_bumper && pinchTimer.milliseconds() >= 500){
                        robot.firetruck.setPincherPosition(ConstantsV4.PINCHER_IDLE);
                        pinchState = PinchState.BUTTON_END;
                    }
                    break;
                case BUTTON_END:
                    if(!gamepad2.right_bumper){
                        pinchState = PinchState.BUTTON_START;
                    }
                    break;
                default:
                    pinchState = PinchState.BUTTON_START;
            }
            /*
            if(gamepad2.x) {
                robot.firetruck.driveDrill(-ConstantsV4.DRILL_SPEED);
            } else if(gamepad2.b) {
                robot.firetruck.driveDrill(ConstantsV4.DRILL_SPEED);
            } else {
                robot.firetruck.stopDrill();
            }
            */
            if(gamepad2.dpad_down){
                robot.firetruck.setTilt(0,ConstantsV4.TILT_IDLE_POW);
                robot.firetruck.setLift(0,ConstantsV4.LIFT_IDLE_POW);
                //robot.firetruck.setErectorToIdle();
                robot.firetruck.setPincherToIdle();
            }

            // Manually resets the motor encoders such that the current position becomes the zero point
            if(gamepad2.dpad_up){
                robot.firetruck.tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.firetruck.tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Erector", robot.firetruck.getErectorPos());
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");

            telemetry.addData("Drive Power", "%.2f", axial);
            telemetry.addData("Turn Power", "%.2f", yaw);
            telemetry.addData("Strafe Power", "%.2f", lateral);
            telemetry.addData("Tilt position", robot.firetruck.tilt.getCurrentPosition());
            telemetry.addData("Tilt target", robot.firetruck.tilt.getTargetPosition());
            telemetry.addData("Lift position", robot.firetruck.tilt.getCurrentPosition());
            telemetry.addData("Erector Accumulator", "%.2f", erectorAcc);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            //sleep(50);
        }
    }
}
