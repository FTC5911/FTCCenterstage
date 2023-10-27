package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class collection {


    public collection(HardwareMap hardwareMap) {



        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeSpin = hardwareMap.get(DcMotor.class, "intakeSpin");

        clawLeft.setDirection(Servo.Direction.REVERSE);
        intakeSpin.setDirection(DcMotorSimple.Direction.REVERSE);

        clawRight.setPosition(0);
        clawLeft.setPosition(0);
        intake.setPower(0);
        intakeSpin.setPower(0);
    }

    private Servo clawRight;
    private Servo clawLeft;
    private DcMotor intake;

    private DcMotor intakeSpin;




    public void openClaw(){

        clawRight.setPosition(0);
        clawLeft.setPosition(0);

    }

    public void closeClaw(){
        clawRight.setPosition(1);
        clawLeft.setPosition(1);
    }
    public void grab_pixel(){
        intake.setPower(.3);

    } public void reverse_intake(){
        intake.setPower(-.3);



    } public void spin_up(){
        intakeSpin.setPower(.875);


    } public void spin_down(){
        intakeSpin.setPower(-.875);



    } public void no_y(){
        intakeSpin.setPower(0);

    } public void no_feed(){
        intake.setPower(0);


    }



}