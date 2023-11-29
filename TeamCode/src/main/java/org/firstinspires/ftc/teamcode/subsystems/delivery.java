package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class delivery {


    public delivery(HardwareMap hardwareMap) {


        double p = 0.2;
        double i = 0.0;
        double d = 0.005;

        //liftRight = hardwareMap.get(DcMotorEx.class,"liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");

        //liftRight.setDirection(DcMotorSimple.Direction.FORWARD); //this might not be needed; or the left slide should be the one being reversed
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        plane = hardwareMap.get(Servo.class, "plane");
        keith = hardwareMap.get(Servo.class, "keith");
        hang = hardwareMap.get(DcMotor.class, "hang");
        hanger_mover = hardwareMap.get(Servo.class, "hanger_mover");

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p, i, d);
        reset();

    }

    //private DcMotor liftRight;
    private DcMotor liftLeft;
    private Servo plane;

    private DcMotor hang;

    private Servo keith;

    private Servo hanger_mover;

    private PIDController controller;

    public void extend(double liftSpeed) {

        //liftRight.setPower(Math.abs(liftSpeed));
        liftLeft.setPower(Math.abs(liftSpeed));

    }

    public void retract(double slideSpeed) {

        //liftRight.setPower(-Math.abs(slideSpeed));
        liftLeft.setPower(-Math.abs(slideSpeed));

    }


    public void stall() {

        //liftRight.setPower(0.1);
        liftLeft.setPower(0.08);

    } public void launch(){
        plane.setPosition(1);

    } public void reload(){
        plane.setPosition(0);

    } public void dump(){
        keith.setPosition(0.5);
    } public void back_to_start(){
        keith.setPosition(1);
    } public void gyat(){
        keith.setPosition(0.75);
    }


    public void initiate_hang(){
        hang.setPower(1);
    } public void lower_hang(){
        hang.setPower(-1);
    } public void stop_hang(){
        hang.setPower(0);
    }



    public void ready_to_go(){
        hanger_mover.setPosition(0.1);
    } public void back_to_sleep(){
        hanger_mover.setPosition(0);
    }



    public void moveToStage(String stage) {

        int slidePosition = liftLeft.getCurrentPosition();

        int Position[] = {-75, 1000, 1750, 2500};
        String Stage[] = {"GROUND", "LOW", "MID", "HIGH"};

        int stageIndex = Byte.MAX_VALUE;
        int res = 0;


        for (int i = 0; i < Stage.length; i++) {

            if (Stage[i].equals(stage)) {
                stageIndex = i;
                break;
            }
        }

        if (stageIndex != Byte.MAX_VALUE) {

            res = Position[stageIndex];
            double pid = controller.calculate(slidePosition, res);

            int error = res - slidePosition;

            double power = pid + 0.1;

            if (Math.abs(error) > 100) {

                //liftRight.setPower(power);
                liftLeft.setPower(power);

            } else {

                stall();

            }



        }
    }

    public void reset(){

        //liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}