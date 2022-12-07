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

package org.firstinspires.ftc.teamcode.TeleOpV2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOpV2.RobotHardwareV2;

/**
 * This OpMode Sample illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear opmode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
 */

@TeleOp(name="Finite TeleOp Test", group="Robot")
@Disabled
public class FiniteTeleOpTest extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardwareV2 robot = new RobotHardwareV2(this);

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    LiftState liftState = LiftState.LIFT_START;
    ElapsedTime erectTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        double axial = 0;
        double lateral = 0;
        double yaw = 0;
        double arm = 0;
        double pincherPos = 0;
        double erectorPos = 0;
        final int PINCH_TIME = 500;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        erectTimer.reset();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            switch (liftState){
                case LIFT_START:
                    if(gamepad1.b){
                        robot.setPincherPosition(robot.PINCHER_POS);
                        erectTimer.reset();
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    if(erectTimer.milliseconds() >= PINCH_TIME){
                        robot.setErectorPosition(robot.ERECTOR_POS_UP);
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                case LIFT_DUMP:
                    if(gamepad1.b){
                        robot.setPincherPosition(robot.PINCHER_IDLE);
                        robot.setErectorPosition(robot.ERECTOR_IDLE);
                        erectTimer.reset();
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if(erectTimer.milliseconds() >= PINCH_TIME){
                        liftState = LiftState.LIFT_START;
                    }
                default:
                    liftState = LiftState.LIFT_START;
            }

            if (gamepad1.y && liftState != LiftState.LIFT_START) {
                liftState = LiftState.LIFT_START;
            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            yaw = -gamepad1.left_stick_y * 0.5;  // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x * 0.5;
            axial = gamepad1.right_stick_x * 0.5;

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(axial, lateral, yaw);

            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
            if (gamepad1.right_bumper) {
                robot.setPincherPosition(robot.PINCHER_POS);
            } else if (gamepad1.left_bumper) {
                robot.setPincherPosition(robot.PINCHER_IDLE);
            }

            if (gamepad1.x) {
                robot.setErectorPosition(robot.ERECTOR_POS_UP);
            } else if (gamepad1.b) {
                robot.setErectorPosition(robot.ERECTOR_IDLE);
            }

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
            if (gamepad1.y)
                arm = robot.ARM_UP_POWER;
            else if (gamepad1.a)
                arm = robot.ARM_DOWN_POWER;
            else
                arm = 0;

            robot.setArmPower(arm);

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", axial);
            telemetry.addData("Turn Power", "%.2f", lateral);
            telemetry.addData("Arm Power", "%.2f", arm);
            telemetry.addData("Hand Position", "Offset = %.2f", erectorPos);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
