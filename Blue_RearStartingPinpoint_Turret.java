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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue_RearStartingPinpoint_Turret", group = "Auton")
public class Blue_RearStartingPinpoint_Turret extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    // ---------------- Hardware ----------------
    Servo ballStopper = null; // "ballKick"
    Servo hood = null;        // "hood"

    DcMotor ShooterMotor = null; // "shooter"
    DcMotor IntakeMotor = null;  // "intake"

    DigitalChannel beamBreakSensor = null; // "breakbeam"

    // ------------------- TURRET (FIXED SETPOINTS) -------------------
    private Servo turret = null;
    private static final String TURRET_SERVO_NAME = "turret";

    // Match your TeleOp calibration
    private static final double TURRET_CENTER_POS = 0.50;
    private static final double TURRET_DEG_PER_POS = 355.0;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // BLUE is mirror of RED -> flip sign
    private static final double TURRET_INIT_DEG = 85.0;          // during init
    private static final double TURRET_AFTER_PRELOAD_DEG = 82.0; // after preload scoring
    // ---------------------------------------------------------------

    // ---------------- Shooter PIDF ----------------
    private PIDFController b, s;

    public static double bp = 0.01, bd = 0.0, bf = 0.0,
            sp = 0.01, sd = 0.0001, sf = 0.0;

    double pSwitch = 50;

    // Waits / timing
    double waittime = 0.4;
    double waittime_transfer = 0.3;

    // Drive powers
    double power_pickup = 1.0;
    double power_shooting = 1.0;

    // Intake powers
    double pickupIntakePower = -1.0;   // lane driving
    double searchIntakePower = -0.4;   // while searching (beam not broken) during kicker
    double transferIntakePower = -1.0; // once beam is broken during kicker

    // Kicker positions
    double ballkicker_up = 0.22;
    double ballkicker_down = 0.5;

    // Shooter velocities
    double farvelocity = 1750;
    double nearvelocity = 1750;
    double targetvel = farvelocity;

    // Shoot 3 cycles per scoring event
    private static final int SHOT_CYCLES = 3;

    // Shooter “ready” settings
    private static final double SHOOTER_READY_TOL = 40;        // velocity tolerance (ticks/sec)
    private static final double SHOOTER_READY_TIMEOUT = 2.5;   // seconds to wait before giving up

    // =========================================================
    // BLUE REAR POSES (explicit numbers; mirrored from your RED rear)
    // Mirror rule you’ve been using:
    // x_blue = 144 - x_red
    // y same
    // heading_blue = 180 - heading_red
    //
    // Your RED rear had heading 0deg everywhere -> BLUE becomes 180deg everywhere
    // Park heading 45deg -> BLUE becomes 135deg
    // =========================================================

    // Robot heading stays 180deg (mirror of 0deg)
    private final Pose startPose = new Pose(53, 9, Math.toRadians(180));

    // Keep scoring pose(s) at the wall
    private final Pose scorePose  = new Pose(53, 9, Math.toRadians(180)); // preload score
    private final Pose scorePose1 = new Pose(53, 9, Math.toRadians(180)); // normal shots
    private final Pose scorePose2 = new Pose(53, 9, Math.toRadians(180)); // last shots

    // Lanes mirrored (x only)
    private final Pose pickup1Pose_lane1 = new Pose(48, 35, Math.toRadians(180));
    private final Pose pickup2Pose_lane1 = new Pose(26, 35, Math.toRadians(180));
    private final Pose pickup3Pose_lane1 = new Pose(10, 35, Math.toRadians(180));

    private final Pose pickup1Pose_lane2 = new Pose(48, 63, Math.toRadians(180));
    private final Pose pickup2Pose_lane2 = new Pose(26, 63, Math.toRadians(180));
    private final Pose pickup3Pose_lane2 = new Pose(15, 63, Math.toRadians(180));

    private final Pose pickup1Pose_lane3 = new Pose(48, 84, Math.toRadians(180));
    private final Pose pickup2Pose_lane3 = new Pose(26, 84, Math.toRadians(180));
    private final Pose pickup3Pose_lane3 = new Pose(20, 84, Math.toRadians(180));

    private final Pose parkPose = new Pose(48, 70, Math.toRadians(135));

    // ---------------- Paths ----------------
    private Path scorePreload;

    private PathChain grabPickup1_lane1, grabPickup2_lane1, grabPickup3_lane1, scorePickup1,
            grabPickup1_lane2, grabPickup2_lane2, grabPickup3_lane2, scorePickup2,
            grabPickup1_lane3, grabPickup2_lane3, grabPickup3_lane3, scorePickup3,
            park;

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

    // ------------------- Zero-length path fix helper -------------------
    private static double dist(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }
    // ------------------------------------------------------------------

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

    private boolean shooterAtSpeed(double tol) {
        double v = ((DcMotorEx) ShooterMotor).getVelocity();
        return Math.abs(targetvel - v) <= tol;
    }

    /** Wait until shooter is at speed (or timeout). Keeps PID running while waiting. */
    private void waitForShooterReady(double tol, double timeoutSeconds) {
        ElapsedTime t = new ElapsedTime(SECONDS);
        t.reset();
        while (t.time() < timeoutSeconds && !shooterAtSpeed(tol)) {
            updateShooterPID();
        }
    }

    /** Safe wait that keeps shooter PID running */
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
            updateShooterPID();
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

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        // KEEP ROBOT HEADING STATIC (180deg)
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(180));

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

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_lane1, scorePose1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // lane 2 pickups
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
                .setConstantHeadingInterpolation(Math.toRadians(180))
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

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_lane3, scorePose2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, parkPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.setMaxPower(power_shooting);

                // If preload path is basically zero-length, skip it
                if (dist(startPose, scorePose) < 0.5) {
                    setPathState(1);
                } else {
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;

            case 1:
                follower.setMaxPower(power_shooting);

                if (!follower.isBusy()) {
                    // Don't shoot until shooter reaches velocity
                    waitForShooterReady(SHOOTER_READY_TOL, SHOOTER_READY_TIMEOUT);

                    // preload scoring: ALWAYS 3 cycles
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    // AFTER PRELOAD: turret stays at +78 (mirrored)
                    setTurretDeg(TURRET_AFTER_PRELOAD_DEG);

                    // Start lane 1 pickup: intake ON while driving
                    startLaneIntake();

                    follower.setMaxPower(power_pickup);
                    follower.followPath(grabPickup1_lane1, true);
                    setPathState(2);
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
                follower.setMaxPower(power_pickup);
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
                    waitForShooterReady(SHOOTER_READY_TOL, SHOOTER_READY_TIMEOUT);
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    startLaneIntake();

                    follower.setMaxPower(power_pickup);
                    follower.followPath(grabPickup1_lane2, true);
                    setPathState(6);
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
                follower.setMaxPower(power_pickup);
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
                    waitForShooterReady(SHOOTER_READY_TOL, SHOOTER_READY_TIMEOUT);
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    startLaneIntake();

                    follower.setMaxPower(power_pickup);
                    follower.followPath(grabPickup1_lane3, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2_lane3, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3_lane3, true);
                    setPathState(12);
                }
                opmodeTimer.resetTimer();
                break;

            case 12:
                follower.setMaxPower(power_pickup);

                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2) {
                    stopLaneIntake();

                    follower.setMaxPower(power_shooting);
                    follower.followPath(scorePickup3, true);
                    setPathState(13);
                }
                break;

            case 13:
                follower.setMaxPower(power_shooting);

                if (!follower.isBusy()) {
                    // last shots use near settings
                    targetvel = nearvelocity;
                    hood.setPosition(0.24);

                    waitForShooterReady(SHOOTER_READY_TOL, SHOOTER_READY_TIMEOUT);
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

    @Override
    public void loop() {
        updateShooterPID();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading(deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("beamBroken", isBeamBroken());
        telemetry.addData("intakePower", (IntakeMotor != null) ? IntakeMotor.getPower() : 0.0);
        telemetry.addData("targetVel", targetvel);
        telemetry.addData("shooterVel", ((DcMotorEx) ShooterMotor).getVelocity());
        telemetry.addData("turretInitDeg", TURRET_INIT_DEG);
        telemetry.addData("turretAfterPreloadDeg", TURRET_AFTER_PRELOAD_DEG);
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

        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        // Do NOT raise kicker in init
        ballStopper.setPosition(ballkicker_down);

        hood.setPosition(0.24);

        // intake off until lane pickups
        IntakeMotor.setPower(0.0);

        // INIT: turret -> +78 deg (mirrored)
        setTurretDeg(TURRET_INIT_DEG);

        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        targetvel = farvelocity; // explicit on start
        setPathState(0);
    }

    @Override
    public void stop() {
        super.stop();
        if (IntakeMotor != null) IntakeMotor.setPower(0.0);
        if (ShooterMotor != null) ShooterMotor.setPower(0.0);
    }
}
