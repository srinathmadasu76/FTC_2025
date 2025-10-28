package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.opmode.example;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robotconstants;
import org.firstinspires.ftc.teamcode.config.subsystem.extensionsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.subsystem.liftsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.intakesystem;
import org.firstinspires.ftc.teamcode.config.subsystem.outtakesystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.shootersystem;

import com.qualcomm.robotcore.hardware.DigitalChannel;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "PedroPath_Red_Ver1", group = "Examples")
public class Pedropath_Red_Ver1 extends OpMode {
    VisionSubsystem vision;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    //HardwareMap hardwareMap;
    private liftsystem lift;
    public intakesystem intake;
    private shootersystem shooter;
    public outtakesystem outtake;
    public extensionsystem extension;
    public Servo BigWrist;
    public Servo IntakeServo;
    DigitalChannel breakBeamSensor;
    Servo ballStopper = null;
    DcMotor ShooterMotor = null;
    DcMotor IntakeMotor = null;


    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(139.5, 108, Math.toRadians(0) );

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    //private final Pose scorePose = new Pose(14, 129, Math.toRadians(45));
    private final Pose scorePose = new Pose(109, 90, Math.toRadians(50));
    //private final Pose scorePose = new Pose(19, 111);

    /** Lowest (First) Sample from the Spike Mark */
    //private final Pose pickup1Pose = new Pose(23, 128);
    private final Pose pickup1Pose_lane1 = new Pose(117, 70, Math.toRadians(0));
    private final Pose pickup2Pose_lane1 = new Pose(120, 70, Math.toRadians(0));
    private final Pose pickup3Pose_lane1 = new Pose(123, 70, Math.toRadians(0));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain grabPickup1_lane1, grabPickup2_lane1, grabPickup3_lane1, scorePickup1, scorePickup2, scorePickup3;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        //scorePreload.setConstantInterpolation(startPose.getHeading());
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        //scorePreload.setTangentHeadingInterpolation();


        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /*grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose), new Point(pickup1Pose)))
                //.setTangentHeadingInterpolation()
                //.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

         */

        grabPickup1_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose_lane1)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup2_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose_lane1), new Point(pickup2Pose_lane1)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup3_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose_lane1), new Point(pickup3Pose_lane1)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose_lane1), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose_lane1.getHeading(), scorePose.getHeading())
                //.setTangentHeadingInterpolation()
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                follower.setMaxPower(0.95);
                follower.followPath(scorePreload);

                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                //if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                if(!follower.isBusy()) {
                    safeWaitSeconds(4);
                    IntakeMotor.setPower(-1.);
                    //ballStopper.setPosition(0.2);
                    safeWaitSeconds(4);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1_lane1, true);
                    //follower.followPath(intakePickup1, true);
                    setPathState(2);
                }
                break;

            case 2:

                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    // intake.grab(pathTimer);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //intake.grab(pathTimer);
                    ballStopper.setPosition(0.76);
                    follower.followPath(grabPickup2_lane1, true);
                    // follower.followPath(grabPickup1, true);
                    setPathState(3);
                }

                break;

            case 3:
                if(!follower.isBusy()) {

                    //intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(grabPickup3_lane1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                   // intake.grab(pathTimer);
                    //follower.followPath(scorePickup1, true);
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                   // intake.grab(pathTimer);
                    ballStopper.setPosition(0.2);
                    follower.followPath(scorePickup2, true);
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
        }
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        //ShooterMotor.setPower(-0.5);
        ((DcMotorEx)ShooterMotor).setVelocity(-1000);
        //IntakeMotor.setPower(-1.);
        //shooter.shoot();
        // These loop the movements of the robot
        follower.update();

        autonomousPathUpdate();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        shooter = new shootersystem(hardwareMap);
        intake = new intakesystem(hardwareMap);
        ballStopper = hardwareMap.get(Servo.class,("ballStop"));
        IntakeMotor = hardwareMap.dcMotor.get("intake");
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buildPaths();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


