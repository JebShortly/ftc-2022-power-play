package org.firstinspires.ftc.teamcode.TeleOpV3;

public class Constants {

    // Servo Constants
    public static final double PINCHER_POS =  0.05;     //pincher fully opened
    public static final double PINCHER_IDLE = 0.25;     //pincher fully closed
    public static final double ERECTOR_POS_UP =  0.0;      //erector tilted up
    public static final double ERECTOR_IDLE = 0.2;      //erector level with ground
    public static final double DRILL_IDLE = 0.0;      //drill horizontally centered
    public static final double DRILL_SPEED = -0.4;

    // Joystick Dampering Constants
    public static final double YAW_DAMP = 0.5;
    public static final double LATERAL_DAMP = 0.5;
    public static final double AXIAL_DAMP = 0.5;

    // Lift Constants
    public static final double LIFT_UP_POW = 0.6;
    public static final double LIFT_DOWN_POW = -0.4;

    // Turntable Constants
    public static final double TURNTABLE_POW = 0.45;     //pow for both left and right

    // Tilt Constants
    public static final double TILT_UP_POW = 0.5;
    public static final double TILT_DOWN_POW = -0.1;
    public static final double TILT_IDLE_POW = 0.3;     //pow given to RUN_TO_POSITION to stabilize motor
}
