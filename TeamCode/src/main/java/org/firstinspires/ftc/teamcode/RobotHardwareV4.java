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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The main/hub hardware class where all other subsystems are initialized. Also contains the
 * hardware initializations and helpful methods for the robot drivetrain.
 */

public class RobotHardwareV4 {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor rfMotor = null;
    private DcMotor rbMotor = null;
    private DcMotor lfMotor = null;
    private DcMotor lbMotor = null;

    public FiretruckV4 firetruck;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardwareV4(LinearOpMode opmode) {
        myOpMode = opmode;
        firetruck = new FiretruckV4(opmode);
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors/Servos
        rfMotor = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        rbMotor = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");
        lfMotor = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        lbMotor = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");

        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.FORWARD);
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        firetruck.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral   Right/Left strafing power (-1.0 to 1.0) +ve is right strafe
     * @param yaw       Right/Left turning power (-1.0 to 1.0) +ve is left strafe
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine drive and turn for blended motion.
        double lfPower  = axial + lateral + yaw;
        double rfPower  = axial - lateral - yaw;
        double lbPower  = axial - lateral + yaw;
        double rbPower  = axial + lateral - yaw;

        // Scale the values so neither exceed +/- 1.0
        double max;
        max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        max = Math.max(max, Math.abs(lbPower));
        max = Math.max(max, Math.abs(rbPower));

        if (max > 1.0)
        {
            lfPower /= max;
            rfPower /= max;
            lbPower /= max;
            rbPower /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(rfPower, rbPower, lfPower, lbPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param rfMotor   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rbMotor   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lfMotor   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lbMotor   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double rfMotor, double rbMotor, double lfMotor, double lbMotor) {
        // Output the values to the motor drives.
        this.rfMotor.setPower(rfMotor);
        this.rbMotor.setPower(rbMotor);
        this.lfMotor.setPower(lfMotor);
        this.lbMotor.setPower(lbMotor);
    }

    public void driveForward(double pow){
        this.setDrivePower(pow,pow,pow,pow);
    }
}
