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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;


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
    Servo ballKick = null;
    Servo hood = null;
    private PIDFController b, s;

    private double t = 0;
    public static double bp = 0.03, bd = 0.0, bf = 0.0, sp = 0.01, sd = 0.0001, sf = 0.0;
    double targetvel = 1150;
    double pSwitch = 50;


    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.dcMotor.get("FL")   ;
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        IntakeMotor = hardwareMap.dcMotor.get("intake");
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballKick = hardwareMap.get(Servo.class,("ballKick"));
        hood = hardwareMap.get(Servo.class,("hood"));


        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double slow_down_factor=0.7;
        double slow_down_factor2=1.;



        double intake_motor_power = 1;

        double power_x=0.75;
        double power_y;
        double y;
        double F = 11.7;
        double P = 0.5;
        double I = 0.5;
        double D = 0.5;
        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

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
            // ((DcMotorEx)ShooterMotor).setVelocityPIDFCoefficients(P, I, D, F);
            // ((DcMotorEx)ShooterMotor).setVelocity(1150);
            double currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
            b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
            s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

            if (Math.abs(targetvel - currentvel) < pSwitch) {
                s.updateError(targetvel - currentvel);
                ShooterMotor.setPower(s.run());
            } else {
                b.updateError(targetvel - currentvel);
                ShooterMotor.setPower(b.run());
            }

            //
            //((DcMotorEx)ShooterMotor).setPositionPIDFCoefficients(5.0);

            telemetry.addData("velocity", currentvel);
            telemetry.update();
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
            if(gamepad1.dpad_left)
            {
                hood.setPosition(0.4);

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

                ballKick.setPosition(0.28);
                //safeWaitSeconds(0.4);

                IntakeMotor.setPower(-1.);

            } else if (gamepad1.b) {
                IntakeMotor.setPower(0.);
                // safeWaitSeconds(0.3);
                ballKick.setPosition(0.75);


            }

            if (gamepad1.dpad_right ) {

                // if (timer.time() < 0.8) {
                IntakeMotor.setPower(-1.);
                // }
            }
           // if (gamepad2.left_trigger != 0.0) {


              //  IntakeMotor.setPower(-intake_motor_power);
           // }

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
           // if (gamepad2.dpad_left) {
               // IntakeMotor.setPower(0);
           // }

        }
    }
    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        double currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
        b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));


        double power_x=0.5;
        double power_y,y;
        double slow_down_factor=0.5;
        double slow_down_factor2=1.;
        y = -gamepad1.left_stick_y;
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            if (Math.abs(targetvel - currentvel) < pSwitch) {
                s.updateError(targetvel - currentvel);
                ShooterMotor.setPower(s.run());
            } else {
                b.updateError(targetvel - currentvel);
                ShooterMotor.setPower(b.run());
            }
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

