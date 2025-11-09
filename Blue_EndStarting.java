package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.opmode.example;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name = "Blue_EndStarting", group = "Examples")
public class Blue_EndStarting extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    //HardwareMap hardwareMap;

    public Servo BigWrist;
    public Servo IntakeServo;
    DigitalChannel breakBeamSensor;
//    Servo ballStopper = null;
    DcMotor ShooterMotor = null;
    DcMotor IntakeMotor = null;
    Servo ballStopper = null;
    Servo hood = null;

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
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90) );

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    //private final Pose scorePose = new Pose(14, 129, Math.toRadians(45));
    private final Pose scorePose = new Pose(35, 100, Math.toRadians(155));

    private final Pose scorePose1 = new Pose(35, 100, Math.toRadians(155));


    /** Lowest (First) Sample from the Spike Mark */
    //private final Pose pickup1Pose = new Pose(23, 128);
    private final Pose pickup1Pose_lane1 = new Pose(45, 90, Math.toRadians(-135));
    private final Pose pickup2Pose_lane1 = new Pose(25, 75, Math.toRadians(-135));
    private final Pose pickup3Pose_lane1 = new Pose(10, 75, Math. toRadians(-135));

//    private final Pose scorePickup1 = new Pose()
    private final Pose pickup1Pose_lane2 = new Pose(45, 100, Math.toRadians(-135));
    private final Pose pickup2Pose_lane2 = new Pose(38, 36, Math.toRadians(-135));
    private final Pose pickup3Pose_lane2 = new Pose(48, 36, Math.toRadians(-135));

    private final Pose pickup1Pose_lane3 = new Pose(60, 30, Math.toRadians(-135));
    private final Pose pickup2Pose_lane3 = new Pose(40, 30, Math.toRadians(-135));
    private final Pose pickup3Pose_lane3 = new Pose(35, 30, Math.toRadians(-135));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain grabPickup1_lane1, grabPickup2_lane1, grabPickup3_lane1, scorePickup1,grabPickup2_lane3,grabPickup3_lane3,park, scorePickup2, scorePickup3, grabPickup1_lane2, grabPickup2_lane2, grabPickup3_lane2, grabPickup1_lane3;


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
                .setLinearHeadingInterpolation(pickup1Pose_lane1.getHeading(), pickup2Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup3_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose_lane1), new Point(pickup3Pose_lane1)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup2Pose_lane1.getHeading(), pickup3Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose_lane1), new Point(scorePose1)))
                .setLinearHeadingInterpolation(pickup3Pose_lane1.getHeading(), scorePose1.getHeading())
                //.setTangentHeadingInterpolation()
                .build();

        grabPickup1_lane2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup1Pose_lane2)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose_lane2.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup2_lane2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose_lane2), new Point(pickup2Pose_lane2)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup1Pose_lane2.getHeading(), pickup2Pose_lane2.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup3_lane2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose_lane2), new Point(pickup3Pose_lane2)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup2Pose_lane2.getHeading(), pickup3Pose_lane2.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose_lane2), new Point(scorePose1)))
                .setLinearHeadingInterpolation(pickup3Pose_lane2.getHeading(), scorePose1.getHeading())
                //.setTangentHeadingInterpolation()
                .build();
        grabPickup1_lane3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup1Pose_lane3)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose_lane3.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup2_lane3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose_lane3), new Point(pickup2Pose_lane3)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup1Pose_lane3.getHeading(), pickup2Pose_lane3.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup3_lane3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose_lane3), new Point(pickup3Pose_lane3)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup3Pose_lane3.getHeading(), scorePose1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup1Pose_lane1)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
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
                follower.setMaxPower(0.6);
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

                    IntakeMotor.setPower(-1.);
                    //ballStopper.setPosition(0.2);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
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
//                    ballStopper.setPosition(0.76);
                    follower.followPath(grabPickup2_lane1, true);
                    // follower.followPath(grabPickup1, true);
                    setPathState(4);
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
                    //intake.grab(pathTimer);
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
                    //IntakeMotor.setPower(0.);

                    IntakeMotor.setPower(-1.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    follower.followPath(grabPickup1_lane2, true);
                    setPathState(6);
                }
                break;
            case 6:

                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    // intake.grab(pathTimer);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //intake.grab(pathTimer);
                    //ballStopper.setPosition(0.76);
                    follower.followPath(grabPickup2_lane2, true);
                    // follower.followPath(grabPickup1, true);
                    setPathState(7);
                }

                break;

            case 7:
                if(!follower.isBusy()) {

                    // intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(grabPickup3_lane2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //follower.followPath(scorePickup1, true);
                    follower.followPath(scorePickup1, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //intake.grab(pathTimer);
                    //IntakeMotor.setPower(0.);
                    //safeWaitSeconds(0.3);
                    // ballStopper.setPosition(0.2);

                    IntakeMotor.setPower(-1.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    follower.followPath(grabPickup1_lane3, true);
                    setPathState(11);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //follower.followPath(scorePickup1, true);
                    // follower.followPath(scorePickup1, true);
                    follower.followPath(grabPickup2_lane3,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {

                    // intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(grabPickup3_lane3,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //intake.grab(pathTimer);
                    IntakeMotor.setPower(0.);
                    //safeWaitSeconds(0.3);
                    // ballStopper.setPosition(0.6);
                    //safeWaitSeconds(1.2);
                    IntakeMotor.setPower(-1.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(0.15);
                    ballStopper.setPosition(0.28);
                    //safeWaitSeconds(1.5);
                    follower.followPath(scorePickup1, true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {

                    // intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(park,true);
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
        ((DcMotorEx)ShooterMotor).setVelocity(1000);
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


      //  ballStopper = hardwareMap.get(Servo.class,("ballStop"));
        ballStopper = hardwareMap.get(Servo.class,("ballKick"));
        IntakeMotor = hardwareMap.dcMotor.get("intake");
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        hood = hardwareMap.get(Servo.class,("hood"));
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballStopper.setPosition(0.28);
        hood.setPosition(0.5);
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


