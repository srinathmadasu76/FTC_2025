package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red_FrontStartingPinpoint", group = "Auton")
public class Red_FrontStartingPinpoint extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    Servo ballStopper = null;
    Servo hood = null;
    DcMotor ShooterMotor = null;
    DcMotor ShooterMotor2 = null;
    DcMotor IntakeMotor = null;
    private PIDFController b, s;

    private double t = 0;
    public static double bp = 0.02, bd = 0.0, bf = 0.0, sp = 0.02, sd = 0.0001, sf = 0.0;


    double targetvel = 1700;
    double pSwitch = 50;
    double waittime = 0.55;

    double power_pickup=0.85;
    double power_shooting = 0.95;
    private final Pose startPose = new Pose(120, 120, Math.toRadians(0) );

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    //private final Pose scorePose = new Pose(14, 129, Math.toRadians(45));
    private final Pose scorePose = new Pose(78, 82, Math.toRadians(45));
    private final Pose scorePose1 = new Pose(78, 82, Math.toRadians(45));
    private final Pose scorePose2 = new Pose(78, 82, Math.toRadians(45));
    private final Pose Park = new Pose(78, 64, Math.toRadians(45));
    //private final Pose scorePose = new Pose(19, 111);

    /** Lowest (First) Sample from the Spike Mark */
    //private final Pose pickup1Pose = new Pose(23, 128);
    private final Pose pickup1Pose_lane1 = new Pose(96, 84, Math.toRadians(0));
    private final Pose pickup2Pose_lane1 = new Pose(118, 84, Math.toRadians(0));
    private final Pose pickup3Pose_lane1 = new Pose(122, 84, Math.toRadians(0));//122

    private final Pose pickup1Pose_lane2 = new Pose(96, 60, Math.toRadians(0));
    private final Pose pickup2Pose_lane2 = new Pose(118, 60, Math.toRadians(0));
    private final Pose pickup3Pose_lane2 = new Pose(130, 60, Math.toRadians(0));//122

    private final Pose pickup1Pose_lane3 = new Pose(96, 40, Math.toRadians(0));
    private final Pose pickup2Pose_lane3 = new Pose(118, 40, Math.toRadians(0));
    private final Pose pickup3Pose_lane3 = new Pose(134, 40, Math.toRadians(0));//122

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain grabPickup1_lane1, grabPickup2_lane1, grabPickup3_lane1, scorePickup1, grabPickup1_lane2, grabPickup2_lane2, grabPickup3_lane2, grabPickup1_lane3, grabPickup2_lane3, grabPickup3_lane3, park,scorePickup2, scorePickup3;

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
        scorePreload = new Path(new BezierLine((startPose), (scorePose)));
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
                .addPath(new BezierLine(scorePose, pickup1Pose_lane1))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup2_lane1 = follower.pathBuilder()
                .addPath(new BezierLine((pickup1Pose_lane1), (pickup2Pose_lane1)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup1Pose_lane1.getHeading(), pickup2Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup3_lane1 = follower.pathBuilder()
                .addPath(new BezierLine((pickup2Pose_lane1), (pickup3Pose_lane1)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup2Pose_lane1.getHeading(), pickup3Pose_lane1.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine((pickup3Pose_lane1), (scorePose1)))
                .setLinearHeadingInterpolation(pickup3Pose_lane1.getHeading(), scorePose1.getHeading())
                //.setTangentHeadingInterpolation()
                .build();

        grabPickup1_lane2 = follower.pathBuilder()
                .addPath(new BezierLine((scorePose1), (pickup1Pose_lane2)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose_lane2.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup2_lane2 = follower.pathBuilder()
                .addPath(new BezierLine((pickup1Pose_lane2), (pickup2Pose_lane2)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup1Pose_lane2.getHeading(), pickup2Pose_lane2.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup3_lane2 = follower.pathBuilder()
                .addPath(new BezierLine((pickup2Pose_lane2), (pickup3Pose_lane2)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup2Pose_lane2.getHeading(), pickup3Pose_lane2.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine((pickup3Pose_lane2), (scorePose1)))
                .setLinearHeadingInterpolation(pickup3Pose_lane2.getHeading(), scorePose1.getHeading())
                //.setTangentHeadingInterpolation()
                .build();
        grabPickup1_lane3 = follower.pathBuilder()
                .addPath(new BezierLine((scorePose1), (pickup1Pose_lane3)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose_lane3.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup2_lane3 = follower.pathBuilder()
                .addPath(new BezierLine((pickup1Pose_lane3), (pickup2Pose_lane3)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup1Pose_lane3.getHeading(), pickup2Pose_lane3.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup3_lane3 = follower.pathBuilder()
                .addPath(new BezierLine((pickup2Pose_lane3), (pickup3Pose_lane3)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup2Pose_lane3.getHeading(), pickup3Pose_lane3.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine((pickup3Pose_lane3), (scorePose2)))
                .setLinearHeadingInterpolation(pickup3Pose_lane3.getHeading(), scorePose2.getHeading())
                //.setTangentHeadingInterpolation()
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine((scorePose2), (Park)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose2.getHeading(), Park.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                follower.setMaxPower(power_shooting);
                follower.followPath(scorePreload);
                IntakeMotor.setPower(-1.);
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
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy()) {
                    //safeWaitSeconds(4);
                    IntakeMotor.setPower(0.);

                    ballStopper.setPosition(0.75);

                    safeWaitSeconds(waittime);

                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(-1.);
                    ballStopper.setPosition(0.28);
                    //safeWaitSeconds(3);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1_lane1, true);
                    //follower.followPath(intakePickup1, true);
                    setPathState(2);
                }
                break;

            case 2:

                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    // intake.grab(pathTimer);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //intake.grab(pathTimer);

                    follower.followPath(grabPickup2_lane1, true);
                    // follower.followPath(grabPickup1, true);
                    setPathState(3);
                }

                break;

            case 3:
                if (!follower.isBusy()) {

                    // intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(grabPickup3_lane1, true);
                    setPathState(4);
                }
                break;
            case 4:
                follower.setMaxPower(power_shooting);
                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //follower.followPath(scorePickup1, true);
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //intake.grab(pathTimer);
                    //IntakeMotor.setPower(0.);
                    //safeWaitSeconds(0.3);

                    //safeWaitSeconds(1.2);
                    IntakeMotor.setPower(0.);

                    ballStopper.setPosition(0.75);

                    safeWaitSeconds(waittime);

                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(-1.);
                    ballStopper.setPosition(0.28);
                    //safeWaitSeconds(1.5);
                    follower.followPath(grabPickup1_lane2, true);
                    setPathState(6);
                }
                break;
            case 6:

                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    // intake.grab(pathTimer);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //intake.grab(pathTimer);

                    follower.followPath(grabPickup2_lane2, true);
                    // follower.followPath(grabPickup1, true);
                    setPathState(7);
                }

                break;

            case 7:
                if (!follower.isBusy()) {

                    // intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(grabPickup3_lane2, true);
                    setPathState(8);
                }
                break;
            case 8:
                follower.setMaxPower(power_shooting);
                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //follower.followPath(scorePickup1, true);
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;
            case 9:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //intake.grab(pathTimer);
                    //IntakeMotor.setPower(0.);
                    //safeWaitSeconds(0.3);

                    //safeWaitSeconds(1.2);
                    IntakeMotor.setPower(0.);

                    ballStopper.setPosition(0.75);

                    safeWaitSeconds(waittime);

                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(-1.);
                    ballStopper.setPosition(0.28);
                    //safeWaitSeconds(1.5);
                    follower.followPath(grabPickup1_lane3, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //follower.followPath(scorePickup1, true);
                    // follower.followPath(scorePickup1, true);
                    follower.followPath(grabPickup2_lane3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {

                    // intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(grabPickup3_lane3, true);
                    setPathState(12);
                }
                break;
            case 12:
                follower.setMaxPower(power_shooting);
                if (!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    //intake.grab(pathTimer);
                    //IntakeMotor.setPower(0.);
                    //safeWaitSeconds(0.3);

                    //safeWaitSeconds(1.2);


                    //safeWaitSeconds(1.5);
                    follower.followPath(scorePickup3, true);
                    setPathState(13);
                }
                break;
            case 13:

                if (!follower.isBusy()) {

                    // intake.grab(pathTimer);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    IntakeMotor.setPower(0.);

                    ballStopper.setPosition(0.75);

                    safeWaitSeconds(waittime);

                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    ballStopper.setPosition(0.28);
                    IntakeMotor.setPower(-1.);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(0.);
                    ballStopper.setPosition(0.75);
                    safeWaitSeconds(waittime);
                    IntakeMotor.setPower(-1.);
                    ballStopper.setPosition(0.28);
                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void safeWaitSeconds(double time) {



        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
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
        }
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        //((DcMotorEx)ShooterMotor).setVelocity(-1300);

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

      //  ((DcMotorEx) ShooterMotor2).setVelocity(targetvel);
      //  ((DcMotorEx) ShooterMotor).setVelocity(targetvel);

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        ballStopper = hardwareMap.get(Servo.class,("ballKick"));
        IntakeMotor = hardwareMap.dcMotor.get("intake");
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        ShooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        hood = hardwareMap.get(Servo.class,("hood"));
        //ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballStopper.setPosition(0.28);
        hood.setPosition(0.24);
        ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
