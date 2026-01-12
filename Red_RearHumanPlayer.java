package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Red_RearStartingPinpoint_Turret_HP", group = "Auton")
public class Red_RearHumanPlayer extends OpMode {

    // ---------------- Pedro ----------------
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // REAL timer for state timing (don’t use Pedro Timer for this)
    private final ElapsedTime stateTime = new ElapsedTime(SECONDS);

    // ---------------- Hardware ----------------
    private Servo ballStopper; // "ballKick"
    private Servo hood;        // "hood"
    private Servo turret;      // "turret"

    private DcMotor ShooterMotor; // "shooter"
    private DcMotor IntakeMotor;  // "intake"

    private DigitalChannel beamBreakSensor; // "breakbeam" (KICKER ONLY)

    // ------------------- TURRET -------------------
    private static final String TURRET_SERVO_NAME = "turret";

    private static final double TURRET_CENTER_POS = 0.50;
    private static final double TURRET_DEG_PER_POS = 355.0;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    private static final double TURRET_INIT_DEG = -78.0;
    private static final double TURRET_AFTER_PRELOAD_DEG = -78.0;

    // ---------------- Shooter PIDF ----------------
    private PIDFController b, s;
    public static double bp = 0.01, bd = 0.0, bf = 0.0,
            sp = 0.01, sd = 0.0001, sf = 0.0;

    private double pSwitch = 50;

    // Waits / timing
    private double waittime = 0.40;
    private double waittime_transfer = 0.30;

    // Drive powers
    private double power_pickup = 1.0;
    private double power_shooting = 1.0;

    // Intake powers
    private double pickupIntakePower = -1.0;
    private double searchIntakePower = -0.4;
    private double transferIntakePower = -1.0;

    // Kicker positions
    private double ballkicker_up = 0.22;
    private double ballkicker_down = 0.5;

    // Shooter velocities
    private double farvelocity = 1750;
    private double nearvelocity = 1750;
    private double targetvel = farvelocity;

    // Shoot cycles
    private static final int SHOT_CYCLES = 3;

    // Shooter ready
    private static final double SHOOTER_READY_TOL = 40;
    private static final double SHOOTER_READY_TIMEOUT = 2.5;

    // ---------------- STACK: YOU SAID LANE 3 = Y 35 ----------------
    public static double STACK_Y = 35;

    private final Pose pickup1Pose_stack = new Pose(96,  STACK_Y, Math.toRadians(0));
    private final Pose pickup2Pose_stack = new Pose(118, STACK_Y, Math.toRadians(0));
    private final Pose pickup3Pose_stack = new Pose(134, STACK_Y, Math.toRadians(0));

    // ---------------- HP BEHAVIOR ----------------
    private static final int HP_CYCLES = 3;
    private int hpCycleCount = 0;

    // HP intake is TIMED (NO breakbeam)
    private double hpIntakePower = -1.0;
    private double hpIntakeSeconds = 0.85;

    // HP approach + ram settings
    private double hpApproachTimeout = 2.0;

    private double hpRamPower = 0.55;
    private double hpBackoffPower = 0.55;

    // max time on wall each ram
    private double hpRamMaxStallSeconds = 2.0;
    private double hpBackoffSeconds = 0.22;

    // ---------------- Poses ----------------
    private final Pose startPose = new Pose(91, 9, Math.toRadians(0));

    private final Pose scorePose  = new Pose(91, 9, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(91, 9, Math.toRadians(0));

    // HP poses (tune in visualizer)
    private final Pose hpApproachPose = new Pose(135, 10, Math.toRadians(0));
    private final Pose hpRamPose      = new Pose(136.2, 10, Math.toRadians(0));
    private final Pose hpBackoffPose  = new Pose(135.2, 10, Math.toRadians(0));

    private final Pose parkPose = new Pose(96, 70, Math.toRadians(45));

    // ---------------- Paths ----------------
    private Path scorePreload;

    private PathChain grabPickup1_stack, grabPickup2_stack, grabPickup3_stack, returnFromStack;

    private PathChain goHP_approach, hp_ram1, hp_backoff, hp_ram2, returnFromHP;

    private PathChain park;

    // ------------------- Helpers -------------------
    private void startLaneIntake() { IntakeMotor.setPower(pickupIntakePower); }
    private void stopLaneIntake()  { IntakeMotor.setPower(0.0); }

    // NON-BLOCKING HP intake control
    private void startHpIntake() { IntakeMotor.setPower(hpIntakePower); }
    private void stopHpIntake()  { IntakeMotor.setPower(0.0); }

    private static double turretDegToServoPos(double turretDeg) {
        double servoPos = TURRET_CENTER_POS + (turretDeg / TURRET_DEG_PER_POS);
        return Range.clip(servoPos, SERVO_MIN_POS, SERVO_MAX_POS);
    }

    private void setTurretDeg(double deg) {
        if (turret != null) turret.setPosition(turretDegToServoPos(deg));
    }

    private static double dist(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

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

    private void waitForShooterReady(double tol, double timeoutSeconds) {
        ElapsedTime t = new ElapsedTime(SECONDS);
        t.reset();
        while (t.time() < timeoutSeconds && !shooterAtSpeed(tol)) {
            updateShooterPID();
        }
    }

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

    // ------------------- Build paths -------------------
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));

        double h0 = Math.toRadians(0);

        // STACK (Y=35)
        grabPickup1_stack = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1Pose_stack))
                .setConstantHeadingInterpolation(h0)
                .build();

        grabPickup2_stack = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose_stack, pickup2Pose_stack))
                .setConstantHeadingInterpolation(h0)
                .build();

        grabPickup3_stack = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose_stack, pickup3Pose_stack))
                .setConstantHeadingInterpolation(h0)
                .build();

        returnFromStack = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_stack, scorePose1))
                .setConstantHeadingInterpolation(h0)
                .build();

        // HP approach
        goHP_approach = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, hpApproachPose))
                .setConstantHeadingInterpolation(h0)
                .build();

        // HP ram/backoff/ram
        hp_ram1 = follower.pathBuilder()
                .addPath(new BezierLine(hpApproachPose, hpRamPose))
                .setConstantHeadingInterpolation(h0)
                .build();

        hp_backoff = follower.pathBuilder()
                .addPath(new BezierLine(hpRamPose, hpBackoffPose))
                .setConstantHeadingInterpolation(h0)
                .build();

        hp_ram2 = follower.pathBuilder()
                .addPath(new BezierLine(hpBackoffPose, hpRamPose))
                .setConstantHeadingInterpolation(h0)
                .build();

        returnFromHP = follower.pathBuilder()
                .addPath(new BezierLine(hpRamPose, scorePose1))
                .setConstantHeadingInterpolation(h0)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, parkPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), parkPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
        stateTime.reset(); // <<< CRITICAL: real timer reset on every state change
    }

    private void startHpCycle() {
        follower.setMaxPower(power_pickup);
        follower.followPath(goHP_approach, true);
        setPathState(50);
    }

    // ------------------- State machine -------------------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // 0: preload move (or skip)
            case 0:
                follower.setMaxPower(power_shooting);
                if (dist(startPose, scorePose) < 0.5) {
                    setPathState(1);
                } else {
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;

            // 1: shoot preload, then go stack
            case 1:
                follower.setMaxPower(power_shooting);
                if (!follower.isBusy()) {
                    waitForShooterReady(SHOOTER_READY_TOL, SHOOTER_READY_TIMEOUT);
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    setTurretDeg(TURRET_AFTER_PRELOAD_DEG);

                    startLaneIntake();
                    follower.setMaxPower(power_pickup);
                    follower.followPath(grabPickup1_stack, true);
                    setPathState(2);
                }
                break;

            // 2: stack pickup1 -> pickup2
            case 2:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2_stack, true);
                    setPathState(3);
                }
                break;

            // 3: stack pickup2 -> pickup3
            case 3:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3_stack, true);
                    setPathState(4);
                }
                break;

            // 4: after pickup3 (or timeout), return to score
            case 4:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2.2) {
                    stopLaneIntake();
                    follower.setMaxPower(power_shooting);
                    follower.followPath(returnFromStack, true);
                    setPathState(5);
                }
                break;

            // 5: shoot after stack, then HP cycles
            case 5:
                follower.setMaxPower(power_shooting);
                if (!follower.isBusy()) {
                    waitForShooterReady(SHOOTER_READY_TOL, SHOOTER_READY_TIMEOUT);
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    hpCycleCount = 0;
                    startHpCycle();
                }
                break;

            // ---------------- HP: approach ----------------
            case 50:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy() || stateTime.time() > hpApproachTimeout) {
                    follower.setMaxPower(hpRamPower);
                    follower.followPath(hp_ram1, true);
                    setPathState(51);
                }
                break;

            // ---------------- HP: RAM #1 (TIME ONLY, MAX 2s) ----------------
            case 51:
                follower.setMaxPower(hpRamPower);
                if (stateTime.time() > hpRamMaxStallSeconds) {
                    follower.setMaxPower(hpBackoffPower);
                    follower.followPath(hp_backoff, true);
                    setPathState(52);
                }
                break;

            // ---------------- HP: BACKOFF (TIME ONLY) ----------------
            case 52:
                follower.setMaxPower(hpBackoffPower);
                if (stateTime.time() > hpBackoffSeconds) {
                    follower.setMaxPower(hpRamPower);
                    follower.followPath(hp_ram2, true);
                    setPathState(53);
                }
                break;

            // ---------------- HP: RAM #2 (TIME ONLY, MAX 2s) ----------------
            case 53:
                follower.setMaxPower(hpRamPower);
                if (stateTime.time() > hpRamMaxStallSeconds) {
                    startHpIntake();
                    setPathState(54);
                }
                break;

            // ---------------- HP: intake timed (NON-BLOCKING) ----------------
            case 54:
                if (stateTime.time() > hpIntakeSeconds) {
                    stopHpIntake();

                    follower.setMaxPower(power_shooting);
                    follower.followPath(returnFromHP, true);
                    setPathState(55);
                }
                break;

            // ---------------- HP: return -> shoot -> repeat ----------------
            case 55:
                follower.setMaxPower(power_shooting);
                if (!follower.isBusy()) {
                    waitForShooterReady(SHOOTER_READY_TOL, SHOOTER_READY_TIMEOUT);
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    hpCycleCount++;
                    if (hpCycleCount < HP_CYCLES) {
                        startHpCycle();
                    } else {
                        follower.followPath(park, true);
                        setPathState(-1);
                    }
                }
                break;

            case -1:
                if (IntakeMotor != null) IntakeMotor.setPower(0.0);
                break;
        }
    }

    @Override
    public void loop() {
        updateShooterPID();
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("STACK_Y", STACK_Y);
        telemetry.addData("pathState", pathState);
        telemetry.addData("stateTime", stateTime.time());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("hpCycle", hpCycleCount + " / " + HP_CYCLES);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading(deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("beamBroken(kickerOnly)", isBeamBroken());
        telemetry.addData("targetVel", targetvel);
        telemetry.addData("shooterVel", ((DcMotorEx) ShooterMotor).getVelocity());
        telemetry.addData("intakePower", (IntakeMotor != null) ? IntakeMotor.getPower() : 0.0);
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
        hood = hardwareMap.get(Servo.class, "hood");
        turret = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);

        IntakeMotor = hardwareMap.dcMotor.get("intake");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterMotor = hardwareMap.dcMotor.get("shooter");

        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        ballStopper.setPosition(ballkicker_down);
        hood.setPosition(0.24);
        IntakeMotor.setPower(0.0);

        setTurretDeg(TURRET_INIT_DEG);

        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

        stateTime.reset();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        targetvel = farvelocity;
        hpCycleCount = 0;
        setPathState(0);
    }

    @Override
    public void stop() {
        super.stop();
        if (IntakeMotor != null) IntakeMotor.setPower(0.0);
        if (ShooterMotor != null) ShooterMotor.setPower(0.0);
    }
}
