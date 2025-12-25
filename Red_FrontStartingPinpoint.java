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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

    // BREAK BEAM
    DigitalChannel beamBreakSensor = null;

    private PIDFController b, s;

    public static double bp = 0.02, bd = 0.0, bf = 0.0, sp = 0.02, sd = 0.0001, sf = 0.0;

    double pSwitch = 50;

    double waittime = 0.15;
    double waittime_transfer = 0.25;

    double power_pickup = 0.5;   // keep your original pickup power
    double power_shooting = 1.0;

    double ballkicker_up = 0.22;
    double ballkicker_down = 0.5;

    double farvelocity = 1550;
    double nearvelocity = 1100;
    double targetvel = nearvelocity;

    // Intake powers (same behavior as Blue)
    private final double laneIntakePower = -1.0;   // when driving lanes
    private final double beamFalsePower = -0.4;    // ONLY during kicker actions if beam is NOT broken

    // "stuck" escape timeouts (seconds)
    private final double moveTimeoutSeconds = 3.0;

    private final Pose startPose = new Pose(120, 120, Math.toRadians(0));

    /** Scoring Pose of our robot. */
    private final Pose scorePose  = new Pose(96, 96, Math.toRadians(45)); // preload
    private final Pose scorePose1 = new Pose(96, 96, Math.toRadians(45)); // lane 1 & lane 2 scoring

    // CHANGE YOU ASKED FOR:
    // lane 3 shooting position SAME as lane 1 & lane 2 (scorePose1)
    private final Pose scorePose2 = new Pose(scorePose1.getX(), scorePose1.getY(), scorePose1.getHeading());

    private final Pose Park = new Pose(94, 68, Math.toRadians(45));

    private final Pose pickup1Pose_lane1 = new Pose(96, 87, Math.toRadians(0));
    private final Pose pickup2Pose_lane1 = new Pose(118, 87, Math.toRadians(0));
    private final Pose pickup3Pose_lane1 = new Pose(127, 87, Math.toRadians(0));

    private final Pose pickup1Pose_lane2 = new Pose(96, 62, Math.toRadians(0));
    private final Pose pickup2Pose_lane2 = new Pose(118, 62, Math.toRadians(0));
    private final Pose pickup3Pose_lane2 = new Pose(124, 62, Math.toRadians(0));

    private final Pose pickup1Pose_lane3 = new Pose(96, 40, Math.toRadians(0));
    private final Pose pickup2Pose_lane3 = new Pose(118, 40, Math.toRadians(0));
    private final Pose pickup3Pose_lane3 = new Pose(124, 40, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1_lane1, grabPickup2_lane1, grabPickup3_lane1, scorePickup1,
            grabPickup1_lane2, grabPickup2_lane2, grabPickup3_lane2,
            grabPickup1_lane3, grabPickup2_lane3, grabPickup3_lane3,
            park, scorePickup2, scorePickup3;

    // ------------------- Breakbeam helpers -------------------
    // Active-low breakbeam on DigitalChannel:
    // getState()==true  -> unbroken
    // getState()==false -> broken
    private boolean isBeamBroken() {
        return beamBreakSensor != null && !beamBreakSensor.getState();
    }

    // - If beam is broken -> intake full (-1.0)
    // - If beam NOT broken -> intake gentle (-0.4)
    // - Only applies during the kicker action
    private void applyIntakeForBeamDuringKicker() {
        if (isBeamBroken()) {
            IntakeMotor.setPower(laneIntakePower);
        } else {
            IntakeMotor.setPower(beamFalsePower);
        }
    }

    // One kicker "cycle": down -> run intake based on beam -> transfer wait -> intake off
    private void kickerCycleWithBeam() {
        ballStopper.setPosition(ballkicker_down);
        safeWaitSeconds(waittime);

        applyIntakeForBeamDuringKicker();
        safeWaitSeconds(waittime_transfer);

        IntakeMotor.setPower(0.0);
    }

    // Start lane intake while we drive pickup paths
    private void startLaneIntake() {
        IntakeMotor.setPower(laneIntakePower);
    }

    // Stop lane intake (usually right when we start scoring)
    private void stopIntake() {
        IntakeMotor.setPower(0.0);
    }
    // ---------------------------------------------------------

    // --------- "stuck" / fail-safe helper ----------
    private void failSafeAdvanceIfStuck(double timeoutSeconds, int nextState) {
        if (opmodeTimer.getElapsedTimeSeconds() > timeoutSeconds) {
            try { follower.breakFollowing(); } catch (Exception ignored) { }
            setPathState(nextState);
        }
    }
    // -----------------------------------------------

    public void buildPaths() {
        // DO NOT change pathing order, only scorePickup3 end pose is now scorePose2 == scorePose1

        scorePreload = new Path(new BezierLine((startPose), (scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1_lane1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose_lane1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose_lane1.getHeading())
                .build();

        grabPickup2_lane1 = follower.pathBuilder()
                .addPath(new BezierLine((pickup1Pose_lane1), (pickup2Pose_lane1)))
                .setLinearHeadingInterpolation(pickup1Pose_lane1.getHeading(), pickup2Pose_lane1.getHeading())
                .build();

        grabPickup3_lane1 = follower.pathBuilder()
                .addPath(new BezierLine((pickup2Pose_lane1), (pickup3Pose_lane1)))
                .setLinearHeadingInterpolation(pickup2Pose_lane1.getHeading(), pickup3Pose_lane1.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine((pickup3Pose_lane1), (scorePose1)))
                .setLinearHeadingInterpolation(pickup3Pose_lane1.getHeading(), scorePose1.getHeading())
                .build();

        grabPickup1_lane2 = follower.pathBuilder()
                .addPath(new BezierLine((scorePose1), (pickup1Pose_lane2)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose_lane2.getHeading())
                .build();

        grabPickup2_lane2 = follower.pathBuilder()
                .addPath(new BezierLine((pickup1Pose_lane2), (pickup2Pose_lane2)))
                .setLinearHeadingInterpolation(pickup1Pose_lane2.getHeading(), pickup2Pose_lane2.getHeading())
                .build();

        grabPickup3_lane2 = follower.pathBuilder()
                .addPath(new BezierLine((pickup2Pose_lane2), (pickup3Pose_lane2)))
                .setLinearHeadingInterpolation(pickup2Pose_lane2.getHeading(), pickup3Pose_lane2.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine((pickup3Pose_lane2), (scorePose1)))
                .setLinearHeadingInterpolation(pickup3Pose_lane2.getHeading(), scorePose1.getHeading())
                .build();

        grabPickup1_lane3 = follower.pathBuilder()
                .addPath(new BezierLine((scorePose1), (pickup1Pose_lane3)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose_lane3.getHeading())
                .build();

        grabPickup2_lane3 = follower.pathBuilder()
                .addPath(new BezierLine((pickup1Pose_lane3), (pickup2Pose_lane3)))
                .setLinearHeadingInterpolation(pickup1Pose_lane3.getHeading(), pickup2Pose_lane3.getHeading())
                .build();

        grabPickup3_lane3 = follower.pathBuilder()
                .addPath(new BezierLine((pickup2Pose_lane3), (pickup3Pose_lane3)))
                .setLinearHeadingInterpolation(pickup2Pose_lane3.getHeading(), pickup3Pose_lane3.getHeading())
                .build();

        // CHANGE: end pose is now scorePose2 == scorePose1 (same XY + same heading)
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine((pickup3Pose_lane3), (scorePose2)))
                .setLinearHeadingInterpolation(pickup3Pose_lane3.getHeading(), scorePose2.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine((scorePose2), (Park)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), Park.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                follower.setMaxPower(power_shooting);

                if (!follower.isBusy()) {
                    stopIntake();

                    // Preload scoring sequence (beam logic EVERY cycle)
                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);
                    ballStopper.setPosition(ballkicker_down);

                    // Lane intake ON
                    startLaneIntake();

                    follower.followPath(grabPickup1_lane1, true);
                    setPathState(3);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 1);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    startLaneIntake();
                    follower.followPath(grabPickup2_lane1, true);
                    setPathState(3);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    startLaneIntake();
                    follower.followPath(grabPickup3_lane1, true);
                    setPathState(4);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 4);
                }
                opmodeTimer.resetTimer();
                break;

            case 4:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2) {
                    stopIntake();
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 5);
                }
                break;

            case 5:
                follower.setMaxPower(power_shooting);
                stopIntake();

                if (!follower.isBusy()) {

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    ballStopper.setPosition(ballkicker_down);

                    startLaneIntake();
                    follower.followPath(grabPickup1_lane2, true);
                    setPathState(7);

                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 7);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    startLaneIntake();
                    follower.followPath(grabPickup2_lane2, true);
                    setPathState(7);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    startLaneIntake();
                    follower.followPath(grabPickup3_lane2, true);
                    setPathState(8);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 8);
                }
                opmodeTimer.resetTimer();
                break;

            case 8:
                follower.setMaxPower(power_pickup);
                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2) {
                    stopIntake();
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 9);
                }
                break;

            case 9:
                follower.setMaxPower(power_shooting);
                stopIntake();

                if (!follower.isBusy()) {

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    ballStopper.setPosition(ballkicker_down);

                    startLaneIntake();
                    follower.followPath(grabPickup1_lane3, true);
                    setPathState(11);

                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 11);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    startLaneIntake();
                    follower.followPath(grabPickup2_lane3, true);
                    setPathState(11);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    startLaneIntake();
                    follower.followPath(grabPickup3_lane3, true);
                    setPathState(12);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 12);
                }
                opmodeTimer.resetTimer();
                break;

            case 12:
                follower.setMaxPower(power_pickup);

                // last shooting velocity matches others
                targetvel = nearvelocity;

                // keep PID alive while moving
                double currentvel12 = ((DcMotorEx) ShooterMotor).getVelocity();
                b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
                s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
                if (Math.abs(targetvel - currentvel12) < pSwitch) {
                    s.updateError(targetvel - currentvel12);
                    ShooterMotor.setPower(s.run());
                } else {
                    b.updateError(targetvel - currentvel12);
                    ShooterMotor.setPower(b.run());
                }

                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 2) {
                    stopIntake();
                    follower.followPath(scorePickup3, true);
                    setPathState(13);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, 13);
                }
                break;

            case 13:
                follower.setMaxPower(power_shooting);
                stopIntake();

                // force last cycle to nearvelocity
                targetvel = nearvelocity;

                if (!follower.isBusy()) {

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    kickerCycleWithBeam();

                    ballStopper.setPosition(ballkicker_up);
                    safeWaitSeconds(waittime);

                    ballStopper.setPosition(ballkicker_down);

                    follower.followPath(park, true);
                    setPathState(-1);
                } else {
                    failSafeAdvanceIfStuck(moveTimeoutSeconds, -1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        opmodeTimer.resetTimer(); // per-state timer for fail-safes
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
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
    }

    @Override
    public void loop() {

        // shooter PID
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

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("beamBroken", isBeamBroken());
        telemetry.addData("intakePower", IntakeMotor != null ? IntakeMotor.getPower() : 0.0);
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

        ballStopper = hardwareMap.get(Servo.class, ("ballKick"));
        IntakeMotor = hardwareMap.dcMotor.get("intake");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        ShooterMotor2 = hardwareMap.dcMotor.get("shooter2");

        hood = hardwareMap.get(Servo.class, ("hood"));

        // BREAK BEAM init
        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        ballStopper.setPosition(ballkicker_down);
        hood.setPosition(0.24);

        ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void stop() { }
}
