package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;


public class collection {


    public collection(HardwareMap hardwareMap) {
        double p = 0.2;
        double i = 0.0;
        double d = 0.005;



        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeSpin = hardwareMap.get(DcMotor.class, "intakeSpin");

        clawLeft.setDirection(Servo.Direction.REVERSE);
        intakeSpin.setDirection(DcMotorSimple.Direction.FORWARD);

        clawRight.setPosition(0);
        clawLeft.setPosition(0);
        intake.setPower(0);
        intakeSpin.setPower(0);

        controller = new PIDController(p, i, d);
    }

    private Servo clawRight;
    private Servo clawLeft;
    private DcMotor intake;

    private DcMotor intakeSpin;

    private PIDController controller;













    public void dont_move(){
        intakeSpin.setPower(0.1);
    }

    public void openClaw(){

        clawRight.setPosition(0);
        clawLeft.setPosition(0);

    }

    public void closeClaw(){
        clawRight.setPosition(1);
        clawLeft.setPosition(1);
    }
    public void grab_pixel(){
        intake.setPower(1);

    } public void reverse_intake(){
        intake.setPower(-1);



    } public void spin_up(){
        intakeSpin.setPower(.7);


    } public void spin_down(){
        intakeSpin.setPower(-7);



    } public void no_y(){
        intakeSpin.setPower(0);

    } public void no_feed(){
        intake.setPower(0);


    }
    public void moveToStage(String stage) {

        int slidePosition = intakeSpin.getCurrentPosition();

        int Position[] = {500, 1350};
        String Stage[] = {"GROUND", "LOW"};

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

            if (Math.abs(error) > 0) {

                //liftRight.setPower(power);
                intakeSpin.setPower(power);

            } else {

                dont_move();

            }



        }
    }



}