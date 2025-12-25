package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "teleop_blue")
public class Teleop_2025_Alex extends LinearOpMode {

    // ---------------- Hardware ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intakeMotor;
    private DcMotor shooterMotor;
    private Servo ballKick, hood;
    private DigitalChannel beamBreakSensor;
    private Limelight3A limelight;

    // ---------------- Shooter PID ----------------
    private PIDFController b1, s1;

    public static double bp = 0.01, bd = 0.0, bf = 0.0;
    public static double sp = 0.01, sd = 0.0001, sf = 0.0;

    private double targetVel = 1700;
    private final double pSwitch = 50;

    private final double farVelocity = 1550;
    private final double nearVelocity = 1300;

    private boolean shooterEnabled = false;

    // Limelight aim (TAG = 20)
    private final int AIM_TAG_ID = 20;
    private final double AIM_DEADBAND_DEG = 2.5;
    private final double AIM_KP = 0.015;
    private final double AIM_TURN_MAX = 0.25;

    // Kicker positions
    private final double ballKickerUp = 0.22;
    private final double ballKickerDown = 0.5;

    // Intake behavior
    private boolean intakeArmed = false;      // X arms, B disarms, JAM also disarms
    private final double INTAKE_POWER = -1.0;

    // Jam detect
    private final double CURRENT_LIMIT = 2.0;
    private final long SPIKE_TIME_MS = 1000;
    private long jamStartTimeMs = 0;
    private boolean jamTiming = false;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft   = hardwareMap.dcMotor.get("BL");
        backRight  = hardwareMap.dcMotor.get("BR");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor = hardwareMap.dcMotor.get("shooter");

        ballKick = hardwareMap.get(Servo.class, "ballKick");
        hood     = hardwareMap.get(Servo.class, "hood");

        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        // KEEP YOUR EXACT MOTOR DIRECTIONS
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter PID
        b1 = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s1 = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight3A");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Init
        setDrivePower(0, 0, 0, 0);
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);

        shooterEnabled = false;
        intakeArmed = false;
        jamTiming = false;

        waitForStart();

        while (opModeIsActive()) {

            // Beam broken = TRUE when object blocks beam (your inversion)
            boolean beamBroken = !beamBreakSensor.getState();

            // ---------------- Kicker + Intake latch ----------------
            if (gamepad1.x) {
                ballKick.setPosition(ballKickerDown);
                intakeArmed = true;     // arm intake
                jamTiming = false;
            }

            if (gamepad1.b) {
                ballKick.setPosition(ballKickerUp);
                intakeArmed = false;    // disarm intake
                intakeMotor.setPower(0.0);
                jamTiming = false;
            }

            // Intake runs ONLY when armed AND beam is broken
            boolean shouldRunIntake = intakeArmed && beamBroken;

            if (shouldRunIntake) {
                intakeMotor.setPower(INTAKE_POWER);

                // Jam detection while intake is running
                double amps = intakeMotor.getCurrent(CurrentUnit.AMPS);

                if (amps > CURRENT_LIMIT && !jamTiming) {
                    jamTiming = true;
                    jamStartTimeMs = System.currentTimeMillis();
                }

                if (jamTiming && amps > CURRENT_LIMIT
                        && (System.currentTimeMillis() - jamStartTimeMs) >= SPIKE_TIME_MS) {
                    // JAM: stop and DISARM until next X press
                    intakeMotor.setPower(0.0);
                    intakeArmed = false;     // <<< key change you asked for
                    jamTiming = false;
                }

                if (amps <= CURRENT_LIMIT) {
                    jamTiming = false;
                }

            } else {
                intakeMotor.setPower(0.0);
                jamTiming = false;
            }

            // ---------------- Hood presets + Shooter enable ----------------
            if (gamepad1.dpad_left) {
                hood.setPosition(0.24);
                targetVel = nearVelocity;
                shooterEnabled = true;
            }
            if (gamepad1.dpad_right) {
                hood.setPosition(0.24);
                targetVel = farVelocity;
                shooterEnabled = true;
            }

            // ---------------- Drive inputs ----------------
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            if (gamepad1.right_bumper) strafe = 1.0;
            if (gamepad1.left_bumper)  strafe = -1.0;

            if (gamepad1.y) drive = 1.0;
            if (gamepad1.a) drive = -1.0;

            if (gamepad1.left_trigger > 0.05)  strafe = -1.0;
            if (gamepad1.right_trigger > 0.05) strafe = 1.0;

            // ---------------- Limelight aim (dpad_down aims + shooter ON) ----------------
            if (gamepad1.dpad_down) {
                shooterEnabled = true;

                AimData aim = getAimData(AIM_TAG_ID);
                if (aim != null) {
                    telemetry.addData("AimTag", AIM_TAG_ID);
                    telemetry.addData("TagX(deg)", aim.tagXDeg);

                    if (Math.abs(aim.tagXDeg) > AIM_DEADBAND_DEG) {
                        turn = clamp(aim.tagXDeg * AIM_KP, -AIM_TURN_MAX, AIM_TURN_MAX);
                    }
                } else {
                    telemetry.addData("Aim", "No tag");
                }
            }

            // Mecanum mix + normalize
            double fl = drive + strafe + turn;
            double fr = drive - strafe - turn;
            double bl = drive - strafe + turn;
            double br = drive + strafe - turn;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max; fr /= max; bl /= max; br /= max;

            setDrivePower(fl, bl, fr, br);

            // ---------------- Shooter velocity hold ----------------
            double currentVel = ((DcMotorEx) shooterMotor).getVelocity();

            if (shooterEnabled) {
                b1.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
                s1.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

                double error = targetVel - currentVel;
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

            // Telemetry
            telemetry.addData("BeamBroken", beamBroken);
            telemetry.addData("IntakeArmed", intakeArmed);
            telemetry.addData("IntakeRunning", shouldRunIntake);
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Shooter Vel", currentVel);
            telemetry.addData("Target Vel", shooterEnabled ? targetVel : 0.0);
            telemetry.update();
        }
    }

    private void setDrivePower(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private static class AimData {
        final double tagXDeg;
        final double distanceIn;
        AimData(double tagXDeg, double distanceIn) {
            this.tagXDeg = tagXDeg;
            this.distanceIn = distanceIn;
        }
    }

    private AimData getAimData(int tagId) {
        LLResult res = limelight.getLatestResult();
        if (res == null) return null;

        List<LLResultTypes.FiducialResult> r = res.getFiducialResults();
        if (r == null) return null;

        for (LLResultTypes.FiducialResult f : r) {
            if (f != null && f.getFiducialId() == tagId) {
                double xIn = (f.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch);
                double zIn = (f.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 8;

                Vector e = new Vector();
                e.setOrthogonalComponents(xIn, zIn);

                return new AimData(f.getTargetXDegrees(), e.getMagnitude());
            }
        }
        return null;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
