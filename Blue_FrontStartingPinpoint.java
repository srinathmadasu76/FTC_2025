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

@Autonomous(name = "Blue Front Auto", group = "Auton")
public class  Blue_FrontStartingPinpoint extends OpMode {

    // =========================================================
    // Pedro + State Machine
    // =========================================================
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /**
     * Stall timeout used for EVERY movement/transit state EXCEPT scoring poses.
     * Scoring poses have NO timeout so we don't shoot from a bad pose.
     */
    private static final double STALL_TIMEOUT = 2.0;

    // Simple debug strings for telemetry
    private String lastTransition = "init";
    private String lastPathName = "none";

    // =========================================================
    // Hardware
    // =========================================================
    private Servo ballStopper = null;
    private Servo hood = null;

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

    private DcMotor ShooterMotor = null;
    private DcMotor IntakeMotor = null;

    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    private DigitalChannel beamBreakSensor = null;

    // =========================================================
    // Shooter PID (your existing logic)
    // =========================================================
    private PIDFController b, s;

    public static double bp = 0.02, bd = 0.0, bf = 0.0,
            sp = 0.02, sd = 0.0001, sf = 0.0;

    double pSwitch = 50;

    double waittime = 0.2;
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
    double nearvelocity = 1200;
    double targetvel = nearvelocity;

    // always do 3 cycles every scoring event
    private static final int SHOT_CYCLES = 3;

    // =========================================================
    // UPDATED START + SCORING POSES
    // Start: (25,130) heading 235°
    // Scoring: (54,90) heading 180°
    // =========================================================
    private final Pose startPose = new Pose(25, 130, Math.toRadians(235));

    private final Pose scorePose  = new Pose(54, 90, Math.toRadians(180)); // preload score
    private final Pose scorePose1 = new Pose(54, 90, Math.toRadians(180)); // normal shots
    private final Pose scorePose2 = new Pose(54, 90, Math.toRadians(180)); // last shots

    // Park
    private final Pose Park = new Pose(32, 96, Math.toRadians(140));

    // Lane 1
    private final Pose pickup1Pose_lane1 = new Pose(48, 90, Math.toRadians(180));
    private final Pose pickup2Pose_lane1 = new Pose(22, 90, Math.toRadians(180));
    private final Pose pickup3Pose_lane1 = new Pose(18, 90, Math.toRadians(180));

    // --------- Gate Open POSES (no longer collide with PathChains) ---------
    private final Pose gateOpenPose1 = new Pose(23, 90, Math.toRadians(180));
    private final Pose gateOpenPose2 = new Pose(20, 80, Math.toRadians(180));
    // ----------------------------------------------------------------------------------

    // Lane 2
    private final Pose pickup1Pose_lane2 = new Pose(48, 65, Math.toRadians(180));
    private final Pose pickup2Pose_lane2 = new Pose(19, 65, Math.toRadians(180));
    private final Pose pickup3Pose_lane2 = new Pose(14, 65, Math.toRadians(180));

    // Lane 3
    private final Pose pickup1Pose_lane3 = new Pose(48, 44, Math.toRadians(180));
    private final Pose pickup2Pose_lane3 = new Pose(19, 44, Math.toRadians(180));
    private final Pose pickup3Pose_lane3 = new Pose(14, 44, Math.toRadians(180));

    // =========================================================
    // Paths
    // =========================================================
    private Path scorePreload;

    private PathChain grabPickup1_lane1, grabPickup2_lane1, grabPickup3_lane1,
            gateOpenPath1, gateOpenPath2, scorePickup1,
            grabPickup1_lane2, grabPickup2_lane2, grabPickup3_lane2, scorePickup2,
            grabPickup1_lane3, grabPickup2_lane3, grabPickup3_lane3, scorePickup3,
            park;

    // =========================================================
    // Small helpers (clean + consistent)
    // =========================================================
    private void startLaneIntake() {
        if (IntakeMotor != null) IntakeMotor.setPower(pickupIntakePower);
    }

    private void stopLaneIntake()  {
        if (IntakeMotor != null) IntakeMotor.setPower(0.0);
    }

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

    // ------------------- Shooter helpers -------------------
    private void updateShooterPID() {
        if (ShooterMotor == null) return;

        double currentvel = ((DcMotorEx) ShooterMotor).getVelocity();

        b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        double err = targetvel - currentvel;

        if (Math.abs(err) < pSwitch) {
            s.updateError(err);
            ShooterMotor.setPower(s.run());
        } else {
            b.updateError(err);
            ShooterMotor.setPower(b.run());
        }
    }

    /**
     * Advances state and resets the stall timer.
     * We store "why" so it is obvious in telemetry when we advanced due to stall.
     */
    public void setPathState(int pState, String reason) {
        pathState = pState;
        lastTransition = reason;
        pathTimer.resetTimer();
    }

    // Backwards-compatible call (so you don't have to rewrite everything)
    public void setPathState(int pState) {
        setPathState(pState, "STATE->" + pState);
    }

    private boolean stalled() {
        return pathTimer.getElapsedTimeSeconds() > STALL_TIMEOUT;
    }

    /**
     * For NON-SCORING movement states:
     * advance if path finishes OR stall timeout hits.
     */
    private boolean doneOrStalled() {
        return !follower.isBusy() || stalled();
    }

    private String doneOrStalledReason() {
        return !follower.isBusy() ? "DONE" : "STALLED";
    }

    /**
     * For SCORING poses:
     * advance ONLY when done (NO stall timeout).
     */
    private boolean doneOnly() {
        return !follower.isBusy();
    }
    // ---------------------------------------------------------

    // =========================================================
    // Beam-gated transfer cycle (your existing logic)
    // =========================================================
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
                if (IntakeMotor != null) IntakeMotor.setPower(transferPower);
                return;
            } else {
                if (IntakeMotor != null) IntakeMotor.setPower(searchPower);
            }
            updateShooterPID();
        }

        if (IntakeMotor != null) IntakeMotor.setPower(searchPower);
    }

    private void doBeamGatedTransferCycle(double beamTimeoutSec) {
        if (ballStopper != null) ballStopper.setPosition(ballkicker_up);
        safeWaitSeconds(waittime);

        if (ballStopper != null) ballStopper.setPosition(ballkicker_down);

        waitForBeamThenRunIntake(beamTimeoutSec, searchIntakePower, transferIntakePower);

        safeWaitSeconds(waittime_transfer);

        if (IntakeMotor != null) IntakeMotor.setPower(0.0);
    }

    private void doBeamGatedTransfers(int cycles, double beamTimeoutSec) {
        if (IntakeMotor != null) IntakeMotor.setPower(0.0);

        for (int i = 0; i < cycles; i++) {
            doBeamGatedTransferCycle(beamTimeoutSec);
        }

        if (ballStopper != null) ballStopper.setPosition(ballkicker_down);
        if (IntakeMotor != null) IntakeMotor.setPower(0.0);
    }
    // ---------------------------------------------------------

    // =========================================================
    // Build paths
    // =========================================================
    public void buildPaths() {

        // startPose -> scorePose
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

        // Gate Open Paths
        gateOpenPath1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_lane1, gateOpenPose1))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        gateOpenPath2 = follower.pathBuilder()
                .addPath(new BezierLine(gateOpenPose1, gateOpenPose2))
                .setConstantHeadingInterpolation(pickupHeading)
                .build();

        // score after gate open
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gateOpenPose2, scorePose1))
                .setLinearHeadingInterpolation(gateOpenPose2.getHeading(), scorePose1.getHeading())
                .build();

        // lane 2 pickups (start from scorePose1)
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

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose_lane3, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose_lane3.getHeading(), scorePose2.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, Park))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), Park.getHeading())
                .build();
    }

    // =========================================================
    // Autonomous update
    // =========================================================
    public void autonomousPathUpdate() {

        switch (pathState) {

            // -------------------------------------------------
            // Drive to preload score pose (SCORING pose: no timeout)
            // -------------------------------------------------
            case 0:
                follower.setMaxPower(power_shooting);
                lastPathName = "scorePreload";
                follower.followPath(scorePreload);
                setPathState(1, "START scorePreload");
                break;

            // SCORING POSE: wait ONLY for done (NO STALL TIMEOUT)
            case 1:
                follower.setMaxPower(power_shooting);

                if (doneOnly()) {
                    // scoring preload: ALWAYS 3 cycles
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    // AFTER PRELOAD SCORING: turret -> 45 deg
                    setTurretDeg(TURRET_AFTER_PRELOAD_DEG);

                    // START lane 1 pickup: intake ON while driving the lane
                    startLaneIntake();

                    follower.setMaxPower(power_pickup_1stand3rd);
                    lastPathName = "grabPickup1_lane1";
                    follower.followPath(grabPickup1_lane1, true);
                    setPathState(2, "DONE scorePreload -> START lane1 seg1");
                }
                break;

            // -------------------------------------------------
            // Lane 1 pickups (movement: DONE OR STALL)
            // -------------------------------------------------
            case 2:
                if (doneOrStalled()) {
                    lastPathName = "grabPickup2_lane1";
                    follower.followPath(grabPickup2_lane1, true);
                    setPathState(3, doneOrStalledReason() + " lane1 seg1 -> START seg2");
                }
                break;

            case 3:
                if (doneOrStalled()) {
                    lastPathName = "grabPickup3_lane1";
                    follower.followPath(grabPickup3_lane1, true);
                    // FIX: go to gate open states next (4), do NOT jump to 6
                    setPathState(4, doneOrStalledReason() + " lane1 seg2 -> START seg3");
                }
                break;

            // Gate open 1 (movement: DONE OR STALL)
            case 4:
                if (doneOrStalled()) {
                    stopLaneIntake();
                    lastPathName = "gateOpenPath1";
                    follower.followPath(gateOpenPath1, true);
                    setPathState(5, doneOrStalledReason() + " lane1 seg3 -> START gate1");
                }
                break;

            // Gate open 2 (movement: DONE OR STALL)
            case 5:
                if (doneOrStalled()) {
                    lastPathName = "gateOpenPath2";
                    follower.followPath(gateOpenPath2, true);
                    setPathState(6, doneOrStalledReason() + " gate1 -> START gate2");
                }
                break;

            // Back to score after gate open (movement: DONE OR STALL)
            case 6:
                if (doneOrStalled()) {
                    stopLaneIntake();
                    follower.setMaxPower(power_shooting);
                    lastPathName = "scorePickup1";
                    follower.followPath(scorePickup1, true);
                    setPathState(7, doneOrStalledReason() + " gate2 -> START scorePickup1");
                }
                break;

            // SCORING POSE: wait ONLY for done (NO STALL TIMEOUT)
            case 7:
                follower.setMaxPower(power_shooting);

                if (doneOnly()) {
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    startLaneIntake();

                    follower.setMaxPower(power_pickup_2nd);
                    lastPathName = "grabPickup1_lane2";
                    follower.followPath(grabPickup1_lane2, true);
                    setPathState(8, "DONE scorePickup1 -> START lane2 seg1");
                }
                break;

            // -------------------------------------------------
            // Lane 2 pickups (movement: DONE OR STALL)
            // -------------------------------------------------
            case 8:
                if (doneOrStalled()) {
                    lastPathName = "grabPickup2_lane2";
                    follower.followPath(grabPickup2_lane2, true);
                    setPathState(9, doneOrStalledReason() + " lane2 seg1 -> START seg2");
                }
                break;

            case 9:
                if (doneOrStalled()) {
                    lastPathName = "grabPickup3_lane2";
                    follower.followPath(grabPickup3_lane2, true);
                    setPathState(10, doneOrStalledReason() + " lane2 seg2 -> START seg3");
                }
                break;

            case 10:
                if (doneOrStalled()) {
                    stopLaneIntake();

                    follower.setMaxPower(power_shooting);
                    lastPathName = "scorePickup2";
                    follower.followPath(scorePickup2, true);
                    setPathState(11, doneOrStalledReason() + " lane2 seg3 -> START scorePickup2");
                }
                break;

            // SCORING POSE: wait ONLY for done (NO STALL TIMEOUT)
            case 11:
                follower.setMaxPower(power_shooting);

                if (doneOnly()) {
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    startLaneIntake();

                    follower.setMaxPower(power_pickup_1stand3rd);
                    lastPathName = "grabPickup1_lane3";
                    follower.followPath(grabPickup1_lane3, true);
                    setPathState(12, "DONE scorePickup2 -> START lane3 seg1");
                }
                break;

            // -------------------------------------------------
            // Lane 3 pickups (movement: DONE OR STALL)
            // -------------------------------------------------
            case 12:
                if (doneOrStalled()) {
                    lastPathName = "grabPickup2_lane3";
                    follower.followPath(grabPickup2_lane3, true);
                    setPathState(13, doneOrStalledReason() + " lane3 seg1 -> START seg2");
                }
                break;

            case 13:
                if (doneOrStalled()) {
                    lastPathName = "grabPickup3_lane3";
                    follower.followPath(grabPickup3_lane3, true);
                    setPathState(14, doneOrStalledReason() + " lane3 seg2 -> START seg3");
                }
                break;

            // Movement back to scorePickup3 (NOT scoring yet: DONE OR STALL)
            case 14:
                targetvel = nearvelocity;

                if (doneOrStalled()) {
                    stopLaneIntake();

                    follower.setMaxPower(power_shooting);
                    lastPathName = "scorePickup3";
                    follower.followPath(scorePickup3, true);
                    setPathState(15, doneOrStalledReason() + " lane3 seg3 -> START scorePickup3");
                }
                break;

            // SCORING POSE: wait ONLY for done (NO STALL TIMEOUT)
            case 15:
                targetvel = nearvelocity;

                if (doneOnly()) {
                    doBeamGatedTransfers(SHOT_CYCLES, 1.0);

                    follower.setMaxPower(power_shooting);
                    lastPathName = "park";
                    follower.followPath(park, true);
                    setPathState(16, "DONE scorePickup3 -> START park");
                }
                break;

            // Park (movement: DONE OR STALL) then end
            case 16:
                if (doneOrStalled()) {
                    stopLaneIntake();
                    if (ShooterMotor != null) ShooterMotor.setPower(0.0);
                    setPathState(-1, doneOrStalledReason() + " park -> END");
                }
                break;

            // End state
            case -1:
            default:
                break;
        }
    }

    @Override
    public void loop() {
        updateShooterPID();

        follower.update();
        autonomousPathUpdate();

        // ------------------- better telemetry -------------------
        Pose p = follower.getPose();

        double shooterVel = 0.0;
        double shooterPower = 0.0;
        if (ShooterMotor != null) {
            shooterPower = ShooterMotor.getPower();
            shooterVel = ((DcMotorEx) ShooterMotor).getVelocity();
        }

        double intakePower = (IntakeMotor != null) ? IntakeMotor.getPower() : 0.0;

        telemetry.addLine("=== BLUE FRONT AUTO DEBUG ===");
        telemetry.addData("State", pathState);
        telemetry.addData("Path", lastPathName);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("pathTimer(s)", String.format("%.2f", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Stalled?", stalled());
        telemetry.addData("Last Transition", lastTransition);

        telemetry.addLine("=== POSE ===");
        telemetry.addData("X", String.format("%.1f", p.getX()));
        telemetry.addData("Y", String.format("%.1f", p.getY()));
        telemetry.addData("Heading(deg)", String.format("%.1f", Math.toDegrees(p.getHeading())));

        telemetry.addLine("=== SHOOTER / INTAKE ===");
        telemetry.addData("TargetVel", String.format("%.0f", targetvel));
        telemetry.addData("ShooterVel", String.format("%.0f", shooterVel));
        telemetry.addData("ShooterPower", String.format("%.2f", shooterPower));
        telemetry.addData("BeamBroken", isBeamBroken());
        telemetry.addData("IntakePower", String.format("%.2f", intakePower));
        telemetry.addData("ShotCycles", SHOT_CYCLES);

        telemetry.addLine("=== POSES (sanity) ===");
        telemetry.addData("StartPose", "25,130 @235deg");
        telemetry.addData("ScorePose", "54,90 @180deg");
        telemetry.addData("Gate1", "23,90 @180deg");
        telemetry.addData("Gate2", "20,84 @180deg");

        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        // Build paths AFTER follower exists
        buildPaths();

        follower.setStartingPose(startPose);

        // Map hardware
        ballStopper = hardwareMap.get(Servo.class, "ballKick");
        hood = hardwareMap.get(Servo.class, "hood");

        IntakeMotor = hardwareMap.dcMotor.get("intake");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterMotor = hardwareMap.dcMotor.get("shooter");

        // turret servo
        turret = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        // Safe init positions
        if (ballStopper != null) ballStopper.setPosition(ballkicker_down);
        if (hood != null) hood.setPosition(0.24);

        // default: intake off until we enter lane pickups
        if (IntakeMotor != null) IntakeMotor.setPower(0.0);

        // INIT: turret -> 45 deg
        setTurretDeg(TURRET_INIT_DEG);

        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

        lastTransition = "init complete";
        lastPathName = "none";
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        setPathState(0, "START pressed");
    }

    @Override
    public void stop() {
        super.stop();
        stopLaneIntake();
        if (ShooterMotor != null) ShooterMotor.setPower(0.0);
    }
}
