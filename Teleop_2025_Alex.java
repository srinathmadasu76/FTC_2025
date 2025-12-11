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
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.function.Supplier;
@TeleOp(name="teleop_2025_alex")
public class Teleop_2025_Alex extends  LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor ExtentionMotor = null;
    DcMotorEx IntakeMotor = null;
    DcMotor ShooterMotor = null;
    DcMotor ShooterMotor2 = null;
    Servo ballKick = null;
    Servo hood = null;
    private PIDFController b1, s1, b2, s2;

    private double t = 0;
    public static double bp = 0.01, bd = 0.0, bf = 0.0, sp = 0.01, sd = 0.0001, sf = 0.0;
    double targetvel = 1700;
    double pSwitch = 50;

    double power_rotation = 0.2;
    double angle_positive = 5;
    double angle_negative = -3;
    double farvelocity = 1550;
    double nearvelocity = 1250;
    double ballkicker_up = 0.72;
    double ballkicker_down = 0.28;

    private Follower follower;

    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private Limelight3A limelight;
    double angle;

    private final Pose startPose = new Pose(26, 64, Math.toRadians(180));
    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.dcMotor.get("FL")   ;
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        ShooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        //ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballKick = hardwareMap.get(Servo.class,("ballKick"));
        hood = hardwareMap.get(Servo.class,("hood"));


        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

         ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        double slow_down_factor=0.85;
        double slow_down_factor2=1.;



        double intake_motor_power = 1;
        double CURRENT_LIMIT=2.0;
        long SPIKE_TIME_MS= 1000;
        boolean intakeRunning= false;
        long jamStart=0;

        double power_x=0.75;
        double power_y;
        double y;
        double F = 1.5;
        double P = 0.5;
        double I = 0.5;
        double D = 0.5;

        b1 = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s1 = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
        b2 = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s2 = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));


        FrontRight.setPower(0);
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);

        IntakeMotor.setPower(0);
        ShooterMotor.setPower(0);

        ElapsedTime timer = new ElapsedTime(SECONDS);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 84))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(140), 0.8))
                .build();

        limelight = hardwareMap.get(Limelight3A.class, "limelight3A");
        limelight.setPollRateHz(100);
        // telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        while (opModeIsActive()) {

            //ShooterMotor.setPower(-0.45);
            // ((DcMotorEx)ShooterMotor).setVelocityPIDFCoefficients(P, I, D, F);
            // ((DcMotorEx)ShooterMotor).setVelocity(1150);

            double currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
            b1.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
            s1.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

            if (Math.abs(targetvel - currentvel) < pSwitch) {
                s1.updateError(targetvel - currentvel);
                ShooterMotor.setPower(s1.run());
            } else {
                b1.updateError(targetvel - currentvel);
                telemetry.addData("power1", b1.run());
                ShooterMotor.setPower(b1.run());
            }
            telemetry.addData("velocity1", currentvel);
            telemetry.addData("encoder1", ShooterMotor.getCurrentPosition());
            if (gamepad1.dpad_up) {
                IntakeMotor.setPower(1);
            }
            if (gamepad1.dpad_down) {
                List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();

                LLResultTypes.FiducialResult target = null;
                for (LLResultTypes.FiducialResult i : r) {
                    if (i != null && i.getFiducialId() == 20) {
                        target = i;
                        break;
                    }
                }

                if (target != null) {
                    angle = target.getTargetXDegrees();
                    telemetry.addData("angle", angle);
                    //telemetry.addData("robotheading:", follower::getHeading);
                }
                while(angle>angle_positive)
                {
                    currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
                    b1.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
                    s1.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

                    if (Math.abs(targetvel - currentvel) < pSwitch) {
                        s1.updateError(targetvel - currentvel);
                        ShooterMotor.setPower(s1.run());
                    } else {
                        b1.updateError(targetvel - currentvel);
                        telemetry.addData("power1", b1.run());
                        ShooterMotor.setPower(b1.run());
                    }
                    FrontRight.setPower(-power_rotation);
                    BackLeft.setPower(power_rotation);
                    FrontLeft.setPower(power_rotation);
                    BackRight.setPower(-power_rotation);

                    r = limelight.getLatestResult().getFiducialResults();

                    target = null;
                    for (LLResultTypes.FiducialResult i : r) {
                        if (i != null && i.getFiducialId() == 20) {
                            target = i;
                            break;
                        }
                    }

                    if (target != null) {
                        angle = target.getTargetXDegrees();
                        telemetry.addData("angle", angle);
                        //telemetry.addData("robotheading:", follower::getHeading);
                    }
                }
                while(angle<angle_negative)
                {
                    currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
                    b1.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
                    s1.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

                    if (Math.abs(targetvel - currentvel) < pSwitch) {
                        s1.updateError(targetvel - currentvel);
                        ShooterMotor.setPower(s1.run());
                    } else {
                        b1.updateError(targetvel - currentvel);
                        telemetry.addData("power1", b1.run());
                        ShooterMotor.setPower(b1.run());
                    }
                    FrontRight.setPower(power_rotation);
                    BackLeft.setPower(-power_rotation);
                    FrontLeft.setPower(-power_rotation);
                    BackRight.setPower(power_rotation);

                    r = limelight.getLatestResult().getFiducialResults();

                    target = null;
                    for (LLResultTypes.FiducialResult i : r) {
                        if (i != null && i.getFiducialId() == 20) {
                            target = i;
                            break;
                        }
                    }

                    if (target != null) {
                        angle = target.getTargetXDegrees();
                        telemetry.addData("angle", angle);
                        //telemetry.addData("robotheading:", follower::getHeading);
                    }
                }
            }
/*
            double currentvel2 = ((DcMotorEx)ShooterMotor2).getVelocity();
            b2.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
            s2.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

            if (Math.abs(targetvel - currentvel2) < pSwitch) {
                s2.updateError(targetvel - currentvel2);
                ShooterMotor2.setPower(s2.run());
            } else {
                b2.updateError(targetvel - currentvel2);
                telemetry.addData("power2", b2.run());
                ShooterMotor2.setPower(b2.run());
            }
*/

            // ((DcMotorEx) ShooterMotor2).setVelocity(targetvel);
          //  ((DcMotorEx) ShooterMotor).setVelocity(targetvel);
             //double currentvel2 = ((DcMotorEx)ShooterMotor2).getVelocity();


            double currentvel1 = ((DcMotorEx)ShooterMotor).getVelocity();
            //
            //((DcMotorEx)ShooterMotor).setPositionPIDFCoefficients(5.0);
            telemetry.addData("velocity1", currentvel1);
            //telemetry.addData("velocity2", currentvel2);
           // telemetry.addData("encoder2", ShooterMotor2.getCurrentPosition());
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
                hood.setPosition(0.24);
                targetvel = nearvelocity;

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
                intakeRunning=true;

                ballKick.setPosition(ballkicker_down);
            }
            if(gamepad1.b){
                intakeRunning = false;
                IntakeMotor.setPower(0.0);
                ballKick.setPosition(ballkicker_up);
            }
            if(intakeRunning){
                IntakeMotor.setPower(-1.0);
            }
            else{
                IntakeMotor.setPower(0.0);
            }

            double intakeCurrent = IntakeMotor.getCurrent(CurrentUnit.AMPS);

            if (intakeRunning && intakeCurrent > CURRENT_LIMIT) {
                if (jamStart == 0) {
                    jamStart = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - jamStart >= SPIKE_TIME_MS) {
                    intakeRunning = false;
                    IntakeMotor.setPower(0.0);
                }
            } else {
                jamStart = 0;
            }
            telemetry.addData("Intake Current", intakeCurrent);


                if (gamepad1.dpad_right ) {

                // if (timer.time() < 0.8) {
                //IntakeMotor.setPower(-1.);
                hood.setPosition(0.24);
                targetvel = farvelocity;
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

        b1.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        s1.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
        b2.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        s2.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));



        double power_x=0.5;
        double power_y,y;
        double slow_down_factor=0.5;
        double slow_down_factor2=1.;
        y = -gamepad1.left_stick_y;
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            double currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
            if (Math.abs(targetvel - currentvel) < pSwitch) {
                s1.updateError(targetvel - currentvel);
                ShooterMotor.setPower(s1.run());
            } else {
                b1.updateError(targetvel - currentvel);
                ShooterMotor.setPower(b1.run());
            }

            double currentvel2 = ((DcMotorEx)ShooterMotor).getVelocity();
            if (Math.abs(targetvel - currentvel2) < pSwitch) {
                s2.updateError(targetvel - currentvel2);
                ShooterMotor.setPower(s2.run());
            } else {
                b2.updateError(targetvel - currentvel2);
                ShooterMotor.setPower(b2.run());
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
