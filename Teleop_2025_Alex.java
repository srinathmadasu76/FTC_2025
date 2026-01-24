package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "teleop_blue")
public class Teleop_2025_Alex extends LinearOpMode {

    // ---------------- Hardware ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intakeMotor;
    private DcMotor shooterMotor;
    private Servo ballKick, hood;

    // Turret + Pinpoint
    private GoBildaPinpointDriver pinpoint;
    private Servo turret;

    private DigitalChannel beamBreakSensor;

    // ---------------- Shooter PID ----------------
    private PIDFController b1, s1;

    public static double bp = 0.01, bd = 0.0, bf = 0.0;
    public static double sp = 0.01, sd = 0.0001, sf = 0.0;

    private double targetVelocity = 1700;
    private final double pSwitch = 50;

    private final double farVelocity = 1650;
    private final double nearVelocity = 1300;

    private boolean shooterEnabled = false;

    // ---------------- Kicker ----------------
    private final double ballKickerUp = 0.24;
    private final double ballKickerDown = 0.5;

    // ---------------- Intake behavior ----------------
    private boolean intakeArmed = false;      // LB arms, RB disarms, JAM also disarms
    private final double INTAKE_POWER = -1.0;

    // ---------------- Jam detect ----------------
    private final double CURRENT_LIMIT = 3.0;
    private final long SPIKE_TIME_MS = 1000;
    private long jamStartTimeMs = 0;
    private boolean jamTiming = false;

    // =====================================================================
    //                        TURRET (PINPOINT ONLY)
    // =====================================================================

    private static final String PINPOINT_NAME = "pinpoint";
    private static final String TURRET_SERVO_NAME = "turret";

    // Field target direction (BLUE)
    private static double BLUE_TARGET_DEG = 0.0;

    // Turret limits (deg)
    private static final double TURRET_MIN_DEG = -105.0;
    private static final double TURRET_MAX_DEG =  105.0;

    // Servo limits
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // Flip if turret rotates opposite
    private static int TURRET_DIR = -1;

    // Mechanical trim (deg)
    private static double TURRET_TRIM_DEG = -44.5;

    // Direct mapping calibration
    private static double TURRET_CENTER_POS = 0.4800 ;
    private static double TURRET_DEG_PER_POS = 355.0;

    // Runtime re-zero button state (Y)
    private boolean prevY = false;

    // =====================================================================
    //                 MANUAL OVERRIDE (DPAD RIGHT -> 80deg)
    //                 Velocity change -> resume tracking
    // =====================================================================
    private boolean turretManualOverride = false;
    private double turretManualDeg = 76.0;
    private double overrideVelocityLatch = 0.0;   // velocity at time override was set
    private boolean prevDpadRight = false;

    @Override
    public void runOpMode() {

        // ---------------- Drive ----------------
        frontLeft  = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft   = hardwareMap.dcMotor.get("BL");
        backRight  = hardwareMap.dcMotor.get("BR");

        // NOTE: keep your directions as you had them; if strafe is reversed we can flip later.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Intake ----------------
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- Shooter ----------------
        shooterMotor = hardwareMap.dcMotor.get("shooter");

        // ---------------- Servos ----------------
        ballKick = hardwareMap.get(Servo.class, "ballKick");
        hood     = hardwareMap.get(Servo.class, "hood");

        // ---------------- Beam break ----------------
        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        // ---------------- Shooter PID ----------------
        b1 = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s1 = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

        // ---------------- Turret + Pinpoint ----------------
        turret = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Init outputs
        setDrivePower(0, 0, 0, 0);
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);
        ballKick.setPosition(ballKickerUp);

        shooterEnabled = false;
        intakeArmed = false;
        jamTiming = false;

        // IMPORTANT: turret should NOT move during init -> set once only
        turret.setPosition(Range.clip(TURRET_CENTER_POS, SERVO_MIN_POS, SERVO_MAX_POS));

        telemetry.addLine("teleop_blue: Pinpoint-only turret");
        telemetry.addLine("DPAD_RIGHT: turret -> 80deg (manual). Change velocity preset -> tracking resumes.");
        telemetry.addLine("Press Y during TELEOP to reset Pinpoint and face goal");
        telemetry.addData("BLUE_TARGET_DEG", BLUE_TARGET_DEG);
        telemetry.addData("TURRET_TRIM_DEG", TURRET_TRIM_DEG);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Beam broken = TRUE when object blocks beam
            boolean beamBroken = !beamBreakSensor.getState();

            // =================================================================
            //                        INTAKE + KICKER
            // =================================================================
            if (gamepad1.left_bumper) {
                ballKick.setPosition(ballKickerDown);
                intakeArmed = true;
                jamTiming = false;
            }

            if (gamepad1.right_bumper) {
                ballKick.setPosition(ballKickerUp);
                intakeArmed = false;
                intakeMotor.setPower(0.0);
                jamTiming = false;
            }

            boolean shouldRunIntake = intakeArmed && beamBroken;

            if (shouldRunIntake) {
                intakeMotor.setPower(INTAKE_POWER);

                double amps = intakeMotor.getCurrent(CurrentUnit.AMPS);

                if (amps > CURRENT_LIMIT && !jamTiming) {
                    jamTiming = true;
                    jamStartTimeMs = System.currentTimeMillis();
                }

                if (jamTiming && amps > CURRENT_LIMIT
                        && (System.currentTimeMillis() - jamStartTimeMs) >= SPIKE_TIME_MS) {
                    // JAM: stop and disarm until next LB press
                    intakeMotor.setPower(0.0);
                    intakeArmed = false;
                    jamTiming = false;
                }

                if (amps <= CURRENT_LIMIT) {
                    jamTiming = false;
                }

            } else {
                intakeMotor.setPower(0.0);
                jamTiming = false;
            }

            // =================================================================
            //                     HOOD PRESETS + SHOOTER ENABLE
            // =================================================================

            // Rising-edge detect for DPAD_RIGHT (so we only latch override once per press)
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadRightPressed = dpadRight && !prevDpadRight;
            prevDpadRight = dpadRight;

            if (gamepad1.dpad_left) {
                hood.setPosition(0.24);
                targetVelocity = nearVelocity;
                shooterEnabled = true;
                // When you switch velocity (near vs far), tracking should resume automatically (handled below).
            }

            if (dpadRightPressed) {
                hood.setPosition(0.24);
                targetVelocity = farVelocity;
                shooterEnabled = true;

                // Manual override: turret -> 80deg
                turretManualOverride = true;
                overrideVelocityLatch = targetVelocity; // remember velocity when override was activated

                double manualCmd = Range.clip(turretManualDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
                turret.setPosition(turretDegToServoPos(manualCmd));
            }

            if (gamepad1.dpad_down) {
                shooterEnabled = false;
            }

            // If the selected target velocity changes away from the one that activated override,
            // auto-tracking is allowed again.
            if (turretManualOverride && targetVelocity != overrideVelocityLatch) {
                turretManualOverride = false;
            }

            // =================================================================
            //                             DRIVE (FIXED MECANUM MIX)
            // =================================================================
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            // Standard mecanum mixing
            double fl = drive + strafe + turn;
            double fr = drive - strafe - turn;
            double bl = drive - strafe + turn;
            double br = drive + strafe - turn;

            // Normalize
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            setDrivePower(fl / max, bl / max, fr / max, br / max);

            // =================================================================
            //        RUNTIME RE-ZERO (PRESS Y): zero Pinpoint + face goal
            // =================================================================
            boolean y = gamepad1.y;
            boolean yPressed = y && !prevY; // rising edge detect
            prevY = y;

            if (yPressed) {
                pinpoint.resetPosAndIMU();
                pinpoint.update();

                // Immediately face goal assuming heading is now ~0
                double turretDegFaceGoal =
                        (BLUE_TARGET_DEG - 0.0) * TURRET_DIR + TURRET_TRIM_DEG;

                turretDegFaceGoal = Range.clip(turretDegFaceGoal, TURRET_MIN_DEG, TURRET_MAX_DEG);
                turret.setPosition(turretDegToServoPos(turretDegFaceGoal));
            }

            // =================================================================
            //                     TURRET TRACKING (PINPOINT)
            //   If manual override is active, HOLD 80deg instead of tracking.
            // =================================================================
            double headingDeg = getHeadingDeg();

            double turretDegCmd;
            if (turretManualOverride) {
                turretDegCmd = Range.clip(turretManualDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
            } else {
                turretDegCmd = (BLUE_TARGET_DEG - headingDeg) * TURRET_DIR + TURRET_TRIM_DEG;
                turretDegCmd = Range.clip(turretDegCmd, TURRET_MIN_DEG, TURRET_MAX_DEG);
            }

            turret.setPosition(turretDegToServoPos(turretDegCmd));

            // =================================================================
            //                       SHOOTER VELOCITY HOLD
            // =================================================================
            double currentVel = ((DcMotorEx) shooterMotor).getVelocity();

            if (shooterEnabled) {
                b1.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
                s1.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

                double error = targetVelocity - currentVel;
                if (Math.abs(error) < pSwitch) {
                    s1.updateError(error);
                    shooterMotor.setPower(s1.run());
                } else {
                    b1.updateError(error);
                    shooterMotor.setPower(b1.run());
                }
            } else {
                shooterMotor.setPower(0.0);
            }

            // =================================================================
            //                             TELEMETRY
            // =================================================================
            telemetry.addData("Y Pressed (re-zero)", yPressed);

            telemetry.addData("PP heading(deg)", headingDeg);
            telemetry.addData("TurretManualOverride", turretManualOverride);
            telemetry.addData("TurretDegCmd", turretDegCmd);
            telemetry.addData("TurretPosCmd", turretDegToServoPos(turretDegCmd));

            telemetry.addData("IntakeArmed", intakeArmed);
            telemetry.addData("BeamBroken", beamBroken);
            telemetry.addData("IntakeRunning", shouldRunIntake);

            telemetry.addData("ShooterEnabled", shooterEnabled);
            telemetry.addData("ShooterVel", currentVel);
            telemetry.addData("TargetVel", shooterEnabled ? targetVelocity : 0.0);

            telemetry.update();
        }
    }

    private void setDrivePower(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private double getHeadingDeg() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        return pose.getHeading(AngleUnit.DEGREES);
    }

    private static double turretDegToServoPos(double turretDeg) {
        double servoPos = TURRET_CENTER_POS + (turretDeg / TURRET_DEG_PER_POS);
        return Range.clip(servoPos, SERVO_MIN_POS, SERVO_MAX_POS);
    }
}
