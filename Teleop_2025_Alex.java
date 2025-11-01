package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="teleop_2025_alex")
public class Teleop_2025_Alex extends  LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor ExtentionMotor = null;
    DcMotor IntakeMotor = null;
    DcMotor ShooterMotor = null;
    Servo ballStopper = null;




    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.dcMotor.get("FL")   ;
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        IntakeMotor = hardwareMap.dcMotor.get("intake");
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballStopper = hardwareMap.get(Servo.class,("ballStop"));

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double slow_down_factor=0.7;
        double slow_down_factor2=1.;



        double intake_motor_power = 1;

        double power_x=0.75;
        double power_y;
        double y;


        FrontRight.setPower(0);
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);

        IntakeMotor.setPower(0);
        ShooterMotor.setPower(0);

        ElapsedTime timer = new ElapsedTime(SECONDS);



        waitForStart();
        while (opModeIsActive()) {

            //ShooterMotor.setPower(-0.45);
            ((DcMotorEx)ShooterMotor).setVelocity(-1000);
           // IntakeMotor.setPower(-1.);
            y = -gamepad1.left_stick_y;
            telemetry.addData("Y", y);
            telemetry.update();
            //Strafe Right
            if (gamepad1.right_bumper) {

                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }

            //Strafe Left
            else if (gamepad1.left_bumper) {

                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }


            if (y > 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (y < 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            //Turn antiClockwise
            if (gamepad1.right_stick_x > 0.0) {
                if (gamepad1.right_stick_y > 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }

                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            //Turn Clockwise
            if (gamepad1.right_stick_x < 0.0) {
                if (gamepad1.right_stick_y < 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (gamepad1.y) {
                power_y = 1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (gamepad1.a) {
                power_y = -1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (!gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.right_stick_x == 0.0 && y==0. && !gamepad1.y && !gamepad1.a && gamepad1.left_trigger == 0.0 && gamepad1.right_trigger == 0.0){
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                FrontLeft.setPower(0.0);
                BackRight.setPower(0.0);
            }

            if (gamepad1.x) {

                ballStopper.setPosition(0.2);
                safeWaitSeconds(1.2);

                IntakeMotor.setPower(-1.);

            } else if (gamepad1.b) {
                IntakeMotor.setPower(0.);
                safeWaitSeconds(0.3);
                ballStopper.setPosition(0.85);


            }

            if (gamepad1.dpad_right ) {

                // if (timer.time() < 0.8) {
                IntakeMotor.setPower(-1.);
                // }
            }
            if (gamepad2.left_trigger != 0.0) {


                IntakeMotor.setPower(-intake_motor_power);
            }

            if (gamepad1.left_trigger != 0.0) {
                FrontLeft.setPower(-power_x*5);
                BackLeft.setPower(power_x*5);
                FrontRight.setPower(power_x*5);
                BackRight.setPower(-power_x*5);
            }
            if (gamepad1.right_trigger!=0.0) {
                FrontLeft.setPower(power_x*5);
                BackLeft.setPower(-power_x*5);
                FrontRight.setPower(-power_x*5);
                BackRight.setPower(power_x*5);
            }
            if (gamepad2.dpad_left) {
                IntakeMotor.setPower(0);
            }

        }
    }
    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        double power_x=0.5;
        double power_y,y;
        double slow_down_factor=0.5;
        double slow_down_factor2=1.;
        y = -gamepad1.left_stick_y;
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //Strafe Right
            if (gamepad1.right_bumper) {

                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }

            //Strafe Left
            else if (gamepad1.left_bumper) {

                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }




            if (y > 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (y < 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            //Turn antiClockwise
            if (gamepad1.right_stick_x > 0.0) {
                if (gamepad1.right_stick_y > 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }

                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            //Turn Clockwise
            if (gamepad1.right_stick_x < 0.0) {
                if (gamepad1.right_stick_y < 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (gamepad1.y) {
                power_y = 1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (gamepad1.a) {
                power_y = -1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }



            if (!gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.right_stick_x == 0.0 && y == 0. && !gamepad1.y && !gamepad1.a && gamepad1.left_trigger == 0.0 && gamepad1.right_trigger == 0.0) {
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                FrontLeft.setPower(0.0);
                BackRight.setPower(0.0);
            }
        }
    }
}

