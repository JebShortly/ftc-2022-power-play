package org.firstinspires.ftc.teamcode.TeleOpV3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Firetruck {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //Define motor and servo objects used on firetruck
    private DcMotor turntable = null;
    public DcMotor tilt = null;
    private DcMotor lift = null;
    private Servo pincher = null;
    private Servo erector = null;
    private CRServo drill = null;

    // Define a constructor that allows the OpMode to pass a reference to itself, that way the
    // Firetruck class can access its methods
    public Firetruck(LinearOpMode opMode){
        myOpMode = opMode;
    }

    /**
     * Initialize all of firetruck's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(){
        turntable = myOpMode.hardwareMap.get(DcMotor.class, "turntable");
        tilt = myOpMode.hardwareMap.get(DcMotor.class, "tilt");
        lift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        pincher = myOpMode.hardwareMap.get(Servo.class, "babyMomma");
        erector = myOpMode.hardwareMap.get(Servo.class, "sugarDaddy");
        drill = myOpMode.hardwareMap.get(CRServo.class, "drill");

        turntable.setDirection(DcMotor.Direction.REVERSE);
        tilt.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        this.setPincherToIdle();
        this.setErectorToIdle();
        this.stopDrill();

        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveTurntable(double pow){
        this.turntable.setPower(pow);
    }

    public void driveTurntableLeft(double pow){
        this.driveTurntable(-pow);
    }

    public void driveTurntableRight(double pow){
        this.driveTurntable(pow);
    }

    public void driveTilt(double pow){
        this.tilt.setTargetPosition(tilt.getCurrentPosition());
        this.tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.tilt.setPower(pow);
    }

    public void tiltIdle(double pow){
        this.tilt.setTargetPosition(tilt.getCurrentPosition());
        this.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.tilt.setPower(pow);
    }

    public void setTilt(int pos, double pow){
        this.tilt.setTargetPosition(pos);
        this.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.tilt.setPower(pow);
    }

    public void driveLift(double pow){
        this.lift.setPower(pow);
    }

    public void setPincherPosition(double pos){
        this.pincher.setPosition(pos);
    }

    public void setErectorPosition(double pos){
        this.erector.setPosition(pos);
    }

    public void driveDrill(double pow){
        this.drill.setPower(pow);
    }

    public void setPincherToIdle(){
        setPincherPosition(Constants.PINCHER_IDLE);
    }

    public void setErectorToIdle(){
        setErectorPosition(Constants.ERECTOR_IDLE);
    }
    public void stopDrill(){
        driveDrill(Constants.DRILL_IDLE);
    }



    /*
    -setTurntable
    -setLift
    -setTilt
    -setPincher
    -setErector
    -driveTurntable
    -driveLift
    -driveTilt
     */
}
