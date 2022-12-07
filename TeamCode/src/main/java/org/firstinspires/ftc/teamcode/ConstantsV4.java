package org.firstinspires.ftc.teamcode;

public class ConstantsV4 {

    // Servo Constants
    public static final double PINCHER_POS =  0.15;     //pincher fully lached
    public static final double PINCHER_IDLE = 0.05;     //pincher fully opened
    public static final double ERECTOR_POS_DOWN =  0.9;      //erector tilted down
    public static final double ERECTOR_POS_UP =  0.1;      //erector tilted up
    public static final double ERECTOR_IDLE = 0.5;      //erector level with ground (standard)
    //public static final double ERECTOR_IDLE = 0.0;      //erector level with ground (continuous)
    public static final double DRILL_IDLE = 0.0;      //drill horizontally centered
    public static final double DRILL_SPEED = -0.4;
    public static final double ERECTOR_SPEED = -0.4;

    // Joystick Dampering Constants
    public static final double YAW_DAMP = 0.5;
    public static final double LATERAL_DAMP = 0.5;
    public static final double AXIAL_DAMP = 0.5;

    // Lift Constants
    public static final double LIFT_UP_POW = 0.6;
    public static final double LIFT_DOWN_POW = -0.4;
    public static final double LIFT_IDLE_POW = 0.7;

    // Turntable Constants
    public static final double TURNTABLE_POW = 0.45;     //pow for both left and right

    // Tilt Constants
    public static final double TILT_UP_POW = 0.5;
    public static final double TILT_DOWN_POW = -0.1;
    public static final double TILT_IDLE_POW = 0.7;     //pow given to RUN_TO_POSITION to stabilize motor

    //Auto Constants
    public  static final double FORWARD_POW = 0.3;
}
