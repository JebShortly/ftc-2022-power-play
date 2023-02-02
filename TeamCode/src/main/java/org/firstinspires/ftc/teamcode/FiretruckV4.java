package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOpV3.Constants;

public class FiretruckV4 {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //Define motor and servo objects used on firetruck
    private DcMotor turntable = null;
    public DcMotor tilt = null;
    public DcMotor lift = null;
    private Servo pincher = null;
    private Servo erector = null;
    private CRServo drill = null;

    // Define a constructor that allows the OpMode to pass a reference to itself, that way the
    // Firetruck class can access its methods
    public FiretruckV4(LinearOpMode opMode){
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
        pincher = myOpMode.hardwareMap.get(Servo.class, "drill");
        erector = myOpMode.hardwareMap.get(Servo.class, "sugarDaddy");
        //drill = myOpMode.hardwareMap.get(CRServo.class, "drill");

        turntable.setDirection(DcMotor.Direction.REVERSE);
        tilt.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        this.setPincherToIdle();
        this.setErectorToIdle();
        //this.stopErector();
        //this.stopDrill();

        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getErectorPos(){
        return this.erector.getPosition();
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
    /*
    public void driveLift(double pow){
        this.lift.setPower(pow);
    }
    */
    public void driveLift(double pow){
        this.lift.setTargetPosition(lift.getCurrentPosition());
        this.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.lift.setPower(pow);
    }

    public void liftIdle(double pow){
        this.lift.setTargetPosition(lift.getCurrentPosition());
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setPower(pow);
    }

    public void setLift(int pos, double pow){
        this.lift.setTargetPosition(pos);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setPower(pow);
    }

    public void setPincherPosition(double pos){
        this.pincher.setPosition(pos);
    }

    public void setErectorPosition(double pos){
        this.erector.setPosition(pos);
    }
    /*
    public void driveErector(double pow){
        this.erector.setPower(pow);
    }
    */
    /*
    public void driveDrill(double pow){
        this.drill.setPower(pow);
    }
    */
    public void setPincherToIdle(){
        setPincherPosition(ConstantsV4.PINCHER_IDLE);
    }

    public void setErectorToIdle(){
        setErectorPosition(ConstantsV4.ERECTOR_IDLE);
    }
    /*
    public void stopDrill(){
        driveDrill(ConstantsV4.DRILL_IDLE);
    }
    */
    /*
    public void stopErector(){
        driveErector(ConstantsV4.ERECTOR_IDLE);
    }
    */
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
