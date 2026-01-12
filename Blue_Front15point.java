package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue_Front15point", group = "Auton")
public class Blue_Front15point extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    Servo ballStopper = null;
    Servo hood = null;

    // ------------------- TURRET (AUTO SETPOINTS) -------------------
    private Servo turret = null;
    private static final String TURRET_SERVO_NAME = "turret";

    // Match your TeleOp calibration
    private static final double TURRET_CENTER_POS = 0.50;
    private static final double TURRET_DEG_PER_POS = 355.0;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // Your requested robot-relative angles
    private static final double TURRET_INIT_DEG = 45.0;              // during init
    private static final double TURRET_AFTER_PRELOAD_DEG = 45.0;     // after preload scoring
    // ---------------------------------------------------------------

    DcMotor ShooterMotor = null;
    DcMotor IntakeMotor = null;

    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    DigitalChannel beamBreakSensor = null;

    private PIDFController b, s;

    public static double bp = 0.02, bd = 0.0, bf = 0.0,
            sp = 0.02, sd = 0.0001, sf = 0.0;

    double pSwitch = 50;

    double waittime = 0.17;
    double waittime_transfer = 0.25;

    // Lane pickup powers (drive power)
    double power_pickup_2nd = 1.0;       // lane 2
    double power_pickup_1stand3rd = 1.0; // lane 1 and lane 3
    double power_shooting = 1.0;

    // Intake powers
    double pickupIntakePower = -1.0;     // runs while driving through lane pickups
    double searchIntakePower = -0.4;     // ONLY during kicker cycle if beam is NOT broken
    double transferIntakePower = -1.0;   // during kicker cycle once beam IS broken

    // Kicker
    double ballkicker_up = 0.22;
    double ballkicker_down = 0.5;

    // Shooter velocities
    double farvelocity = 1550;
    double nearvelocity = 1300;
    double targetvel = nearvelocity;

    // always do 3 cycles every scoring event
    private static final int SHOT_CYCLES = 3;

    // =========================================================
    // UPDATED START + SCORING POSES (per your message)
    // Start: (25,130) heading 235°
    // All scoring poses after this: (50,85) heading 180°
    // =========================================================
    private final Pose startPose = new Pose(25, 130, Math.toRadians(235));

    private final Pose scorePose  = new Pose(54, 90, Math.toRadians(180)); // preload score
    private final Pose scorePose1 = new Pose(54, 90, Math.toRadians(180)); // normal shots
    private final Pose scorePose2 = new Pose(54, 90, Math.toRadians(180)); // last shots
    // Park is NOT a scoring pose; leaving as-is unless you tell me new park target
    private final Pose Park = new Pose(32, 96, Math.toRadians(140));

    private final Pose pickup1Pose_lane1 = new Pose(48, 90, Math.toRadians(180));
    private final Pose pickup2Pose_lane1 = new Pose(23, 90, Math.toRadians(180));
    private final Pose pickup3Pose_lane1 = new Pose(20, 90, Math.toRadians(180));

    private final Pose pickup1Pose_lane2 = new Pose(48, 65, Math.toRadians(180));
    private final Pose pickup2Pose_lane2 = new Pose(26, 65, Math.toRadians(180));
    private final Pose pickup3Pose_lane2 = new Pose(24, 65, Math.toRadians(180));

    private final Pose pickup1Pose_lane3 = new Pose(48, 44, Math.toRadians(180));
    private final Pose pickup2Pose_lane3 = new Pose(19, 44, Math.toRadians(180));
    private final Pose pickup3Pose_lane3 = new Pose(15 , 44, Math.toRadians(180));

    private final Pose pickupPose_slider = new Pose(10, 63, Math.toRadians(140));
    private final Pose pushPose_slider = new Pose(13, 65, Math.toRadians(135));

    private Path scorePreload;

    private PathChain grabPickup1_lane1, grabPickup2_lane1, grabPickup3_lane1, scorePickup1,
            grabPickup1_lane2, grabPickup2_lane2, grabPickup3_lane2, scorePickup2,
            grabPickup1_lane3, grabPickup2_lane3, grabPickup3_lane3, scorePickup3,
            park,grabPickup_slider,scorePickupslider,push_slider;

    // ------------------- Intake helpers -------------------
    private void startLaneIntake() { IntakeMotor.setPower(pickupIntakePower); }
    private void stopLaneIntake()  { IntakeMotor.setPower(0.0); }
    // ------------------------------------------------------

    // ------------------- Turret helpers -------------------
    private static double turretDegToServoPos(double turretDeg) {
        double servoPos = TURRET_CENTER_POS + (turretDeg / TURRET_DEG_PER_POS);
        return Range.clip(servoPos, SERVO_MIN_POS, SERVO_MAX_POS);
    }

    private void setTurretDeg(double deg) {
        if (turret != null) turret.setPosition(turretDegToServoPos(deg));
    }
    // ------------------------------------------------------

    // ------------------- Breakbeam helpers -------------------
    // Active-low breakbeam:
    // getState()==true  -> unbroken
    // getState()==false -> broken
    private boolean isBeamBroken() {
        return beamBreakSensor != null && !beamBreakSensor.getState();
    }

    private void updateShooterPID() {
        double currentvel = ((DcMotorEx) ShooterMotor).getVelocity();
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

    private void waitForBeamThenRunIntake(double timeoutSeconds, double searchPower, double transferPower) {
        safeWaitSeconds(0.03);

        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();

        while (timer.time() < timeoutSeconds) {
            if (isBeamBroken()) {
                IntakeMotor.setPower(transferPower);
                return;
            } else {
                IntakeMotor.setPower(searchPower);
            }
            updateShooterPID();
        }

        IntakeMotor.setPower(searchPower);
    }

    private void doBeamGatedTransferCycle(double beamTimeoutSec) {
        ballStopper.setPosition(ballkicker_up);
        safeWaitSeconds(waittime);

        ballStopper.setPosition(ballkicker_down);

        waitForBeamThenRunIntake(beamTimeoutSec, searchIntakePower, transferIntakePower);

        safeWaitSeconds(waittime_transfer);

        IntakeMotor.setPower(0.0);
    }

    private void doBeamGatedTransfers(int cycles, double beamTimeoutSec) {
        IntakeMotor.setPower(0.0);
        for (int i = 0; i < cycles; i++) {
            doBeamGatedTransferCycle(beamTimeoutSec);
        }
        ballStopper.setPosition(ballkicker_down);
        IntakeMotor.setPower(0.0);
    }
    // ---------------------------------------------------------

    public void buildPaths() {

        // UPDATED: startPose -> scorePose (new scoring point)
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // LOCK HEADING ON PICKUPS (pickup heading = 180)
        double pickupHeading = Math.toRadians(180);

        // lane 1 pickups
        grabPickup1_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose_lane1))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        grabPickup2_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose_lane1, pickup2Pose_lane1))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        grabPickup3_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose_lane1, pickup3Pose_lane1))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        // UPDATED: scoring pose is now scorePose1 (50,85,180)
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_lane1, scorePose1))
                .setLinearHeadingInterpolation(pickup3Pose_lane1.getHeading(), scorePose1.getHeading())
                .build();

        // lane 2 pickups (start from scorePose1 which is same scoring point now)
        grabPickup1_lane2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1Pose_lane2))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        grabPickup2_lane2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose_lane2, pickup2Pose_lane2))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        grabPickup3_lane2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose_lane2, pickup3Pose_lane2))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_lane2, scorePose1))
                .setLinearHeadingInterpolation(pickup3Pose_lane2.getHeading(), scorePose1.getHeading())
                .build();

        // lane 3 pickups
        grabPickup1_lane3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1Pose_lane3))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        grabPickup2_lane3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose_lane3, pickup2Pose_lane3))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        grabPickup3_lane3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose_lane3, pickup3Pose_lane3))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        // UPDATED: last scoring pose is scorePose2 (same scoring point now)
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_lane3, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose_lane3.getHeading(), scorePose2.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, Park))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), Park.getHeading())
                .build();

        push_slider= follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pushPose_slider))
                .setLinearHeadingInterpolation(scorePose2.getHeading(),pushPose_slider.getHeading())
                .build();
        grabPickup_slider= follower.pathBuilder()
                .addPath(new BezierLine( pushPose_slider, pickupPose_slider))
                .setLinearHeadingInterpolation( pushPose_slider.getHeading(), pickupPose_slider.getHeading())
                .build();
        scorePickupslider = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose_slider,scorePose2))
                .setLinearHeadingInterpolation(pickupPose_slider.getHeading(), scorePose2.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.setMaxPower(power_shooting);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                follower.setMaxPower(power_shooting);

                if (!follower.isBusy()) {
                    // scoring preload: ALWAYS 3 cycles
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    // AFTER PRELOAD SCORING: turret -> 45 deg
                    setTurretDeg(TURRET_AFTER_PRELOAD_DEG);

                    // START lane 1 pickup: intake ON while driving the lane
                    startLaneIntake();

                    follower.setMaxPower(power_pickup_1stand3rd);
                    follower.followPath(grabPickup1_lane1, true);
                    setPathState(3);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2_lane1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3_lane1, true);
                    setPathState(4);
                }
                opmodeTimer.resetTimer();
                break;

            case 4:
                follower.setMaxPower(power_pickup_1stand3rd);
                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2) {

                    stopLaneIntake();

                    follower.setMaxPower(power_shooting);
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5:
                follower.setMaxPower(power_shooting);

                if (!follower.isBusy()) {
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    startLaneIntake();

                    follower.setMaxPower(power_pickup_2nd);
                    follower.followPath(grabPickup1_lane2, true);
                    setPathState(7);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2_lane2, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3_lane2, true);
                    setPathState(8);
                }
                opmodeTimer.resetTimer();
                break;

            case 8:
                follower.setMaxPower(power_pickup_2nd);
                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2) {

                    stopLaneIntake();

                    follower.setMaxPower(power_shooting);
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;

            case 9:
                follower.setMaxPower(power_shooting);

                if (!follower.isBusy()) {
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    startLaneIntake();

                    follower.setMaxPower(power_pickup_1stand3rd);
                    follower.followPath(push_slider, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup_slider, true);
                    setPathState(11);
                }
                opmodeTimer.resetTimer();
                break;
            case 11:
                follower.setMaxPower(power_pickup_1stand3rd);
                targetvel = nearvelocity;
                safeWaitIntakeSeconds(0.5);
                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 1.5) {

                    //stopLaneIntake();

                    follower.setMaxPower(power_shooting);
                    follower.followPath(scorePickupslider, true);
                    setPathState(12);
                }
                break;

            case 12:
                follower.setMaxPower(power_shooting);

                if (!follower.isBusy()) {
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    startLaneIntake();

                    follower.setMaxPower(power_pickup_1stand3rd);
                    follower.followPath(grabPickup1_lane3, true);
                    setPathState(14);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2_lane3, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3_lane3, true);
                    setPathState(15);
                }
                opmodeTimer.resetTimer();
                break;

            case 15:
                follower.setMaxPower(power_pickup_1stand3rd);
                targetvel = nearvelocity;

                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2) {

                    stopLaneIntake();

                    follower.setMaxPower(power_shooting);
                    follower.followPath(scorePickup3, true);
                    setPathState(16);
                }
                break;

            case 16:
                follower.setMaxPower(power_shooting);
                targetvel = nearvelocity;

                if (!follower.isBusy()) {
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
            updateShooterPID();
        }
    }

    public void safeWaitIntakeSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
            updateShooterPID();
            IntakeMotor.setPower(pickupIntakePower);
        }
    }

    @Override
    public void loop() {
        updateShooterPID();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading(rad)", follower.getPose().getHeading());
        telemetry.addData("heading(deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("beamBroken", isBeamBroken());
        telemetry.addData("intakePower", (IntakeMotor != null) ? IntakeMotor.getPower() : 0.0);
        telemetry.addData("shotCycles", SHOT_CYCLES);

        telemetry.addData("startPose", "25,130 @235deg");
        telemetry.addData("scorePose", "50,85 @180deg");

        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        ballStopper = hardwareMap.get(Servo.class, "ballKick");
        IntakeMotor = hardwareMap.dcMotor.get("intake");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        hood = hardwareMap.get(Servo.class, "hood");

        // turret servo
        turret = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        ballStopper.setPosition(ballkicker_down);
        hood.setPosition(0.24);

        // default: intake off until we enter lane pickups
        IntakeMotor.setPower(0.0);

        // INIT: turret -> 90 deg
        setTurretDeg(TURRET_INIT_DEG);

        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        super.stop();
    }
}

