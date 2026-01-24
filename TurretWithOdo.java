package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Turret Tune (Pinpoint) v3", group="Test")
public class TurretWithOdo extends LinearOpMode {

    // ---------------- Hardware ----------------
    private static final String PINPOINT_NAME = "pinpoint";
    private static final String TURRET_SERVO_NAME = "turret";

    private GoBildaPinpointDriver odo;
    private Servo turret;

    // ---------------- Field targets (match TeleOp) ----------------
    private static double BLUE_TARGET_DEG = 0.0;
    private static double RED_TARGET_DEG  = -100.0;

    // ---------------- Turret limits ----------------
    private static final double TURRET_MIN_DEG = -105.0;
    private static final double TURRET_MAX_DEG =  105.0;

    // ---------------- Servo safety ----------------
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // ---------------- Tunables (COPY INTO TELEOP) ----------------
    private static int TURRET_DIR = -1;
    private static double TURRET_TRIM_DEG = -52.5;

    private static double TURRET_CENTER_POS = 0.4800;
    private static double TURRET_DEG_PER_POS = 355.0;

    // ---------------- Tuning steps ----------------
    private static final double TRIM_STEP_DEG = 0.5;
    private static final double DEG_PER_POS_STEP = 5.0;
    private static final double CENTER_FINE_SCALE = 0.001;

    // ---------------- State ----------------
    private boolean isBlue = true;

    private boolean lastB, lastY, lastX, lastA, lastRB;
    private boolean lastLB;

    // (optional) for telemetry turn-rate feel
    private double lastHeadingDeg = 0.0;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // turret should NOT move during init -> set once only
        turret.setPosition(Range.clip(TURRET_CENTER_POS, SERVO_MIN_POS, SERVO_MAX_POS));

        telemetry.addLine("TURRET TUNER (matches TeleOp turret math)");
        telemetry.addLine("LIMITED TO ±105 DEG");
        telemetry.addLine("B=Blue | Y=Red | X=Flip DIR | A=Rezero");
        telemetry.addLine("RB=Rezero + FACE GOAL");
        telemetry.addLine("DPAD L/R = Trim | DPAD U/D = DegPerPos");
        telemetry.addLine("Left stick X = CenterPos fine adjust");
        telemetry.addLine("Hold LB = freeze turret");
        telemetry.update();

        waitForStart();

        odo.resetPosAndIMU();
        odo.update();

        double holdPos = Range.clip(TURRET_CENTER_POS, SERVO_MIN_POS, SERVO_MAX_POS);

        while (opModeIsActive()) {

            // =======================
            //  Update odo ONCE / LOOP
            // =======================
            odo.update();
            Pose2D pose = odo.getPosition();
            double headingDeg = pose.getHeading(AngleUnit.DEGREES);

            // simple turn-rate estimate (deg/loop) for debugging
            double headingDeltaDeg = AngleUnit.normalizeDegrees(headingDeg - lastHeadingDeg);
            lastHeadingDeg = headingDeg;

            boolean b  = gamepad1.b;
            boolean y  = gamepad1.y;
            boolean x  = gamepad1.x;
            boolean a  = gamepad1.a;
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;

            boolean bPressed  = b && !lastB;
            boolean yPressed  = y && !lastY;
            boolean xPressed  = x && !lastX;
            boolean aPressed  = a && !lastA;
            boolean rbPressed = rb && !lastRB;

            lastB = b;
            lastY = y;
            lastX = x;
            lastA = a;
            lastRB = rb;

            if (bPressed) isBlue = true;
            if (yPressed) isBlue = false;

            if (xPressed) TURRET_DIR *= -1;

            if (aPressed) {
                odo.resetPosAndIMU();
                odo.update();
                // after rezero, also refresh cached heading immediately
                Pose2D p2 = odo.getPosition();
                headingDeg = p2.getHeading(AngleUnit.DEGREES);
                lastHeadingDeg = headingDeg;
            }

            // Rezero + face goal (like your TeleOp Y behavior)
            if (rbPressed) {
                odo.resetPosAndIMU();
                odo.update();

                // assume heading ~0 after reset, but normalize anyway
                double targetFieldDeg = isBlue ? BLUE_TARGET_DEG : RED_TARGET_DEG;
                double errorDeg = AngleUnit.normalizeDegrees(targetFieldDeg - 0.0);

                double turretDegFaceGoal =
                        (errorDeg) * TURRET_DIR + TURRET_TRIM_DEG;

                turretDegFaceGoal = Range.clip(turretDegFaceGoal, TURRET_MIN_DEG, TURRET_MAX_DEG);

                holdPos = turretDegToServoPos(turretDegFaceGoal);
                turret.setPosition(holdPos);

                // refresh cached heading after reset
                Pose2D p2 = odo.getPosition();
                headingDeg = p2.getHeading(AngleUnit.DEGREES);
                lastHeadingDeg = headingDeg;
                headingDeltaDeg = 0.0;
            }

            // Live tuning
            if (gamepad1.dpad_left)  TURRET_TRIM_DEG -= TRIM_STEP_DEG;
            if (gamepad1.dpad_right) TURRET_TRIM_DEG += TRIM_STEP_DEG;

            if (gamepad1.dpad_up)    TURRET_DEG_PER_POS += DEG_PER_POS_STEP;
            if (gamepad1.dpad_down)  TURRET_DEG_PER_POS -= DEG_PER_POS_STEP;

            TURRET_CENTER_POS += gamepad1.left_stick_x * CENTER_FINE_SCALE;

            TURRET_CENTER_POS = Range.clip(TURRET_CENTER_POS, SERVO_MIN_POS, SERVO_MAX_POS);
            TURRET_DEG_PER_POS = Math.max(50.0, TURRET_DEG_PER_POS);

            // Freeze feature
            if (lb && !lastLB) holdPos = turret.getPosition();
            lastLB = lb;

            // =========================
            //   Tracking / Hold output
            // =========================
            double targetFieldDeg = isBlue ? BLUE_TARGET_DEG : RED_TARGET_DEG;

            if (lb) {
                turret.setPosition(holdPos);
            } else {
                // NORMALIZED angle error (prevents wrap weirdness + keeps symmetry)
                double errorDeg = AngleUnit.normalizeDegrees(targetFieldDeg - headingDeg);

                double desiredTurretDegRaw =
                        (errorDeg) * TURRET_DIR + TURRET_TRIM_DEG;

                double desiredTurretDeg =
                        Range.clip(desiredTurretDegRaw, TURRET_MIN_DEG, TURRET_MAX_DEG);

                double servoPos = turretDegToServoPos(desiredTurretDeg);
                turret.setPosition(servoPos);
                holdPos = servoPos;
            }

            // =========================
            //        Telemetry
            // =========================
            telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
            telemetry.addData("TargetFieldDeg", targetFieldDeg);
            telemetry.addData("Heading(deg)", headingDeg);
            telemetry.addData("Heading d(deg/loop)", headingDeltaDeg);

            telemetry.addData("DIR", TURRET_DIR);
            telemetry.addData("Trim(deg)", TURRET_TRIM_DEG);

            telemetry.addData("CenterPos", TURRET_CENTER_POS);
            telemetry.addData("DegPerPos", TURRET_DEG_PER_POS);

            telemetry.addData("TurretServoPos", turret.getPosition());
            telemetry.addData("LB Hold", lb);

            telemetry.addLine("COPY THESE INTO TELEOP:");
            telemetry.addData("TURRET_DIR", TURRET_DIR);
            telemetry.addData("TURRET_TRIM_DEG", TURRET_TRIM_DEG);
            telemetry.addData("TURRET_CENTER_POS", TURRET_CENTER_POS);
            telemetry.addData("TURRET_DEG_PER_POS", TURRET_DEG_PER_POS);
            telemetry.addLine("ALSO SET TELEOP LIMITS: TURRET_MIN_DEG=-90, TURRET_MAX_DEG=+90");
            telemetry.update();
        }
    }

    private static double turretDegToServoPos(double turretDeg) {
        double servoPos = TURRET_CENTER_POS + (turretDeg / TURRET_DEG_PER_POS);
        return Range.clip(servoPos, SERVO_MIN_POS, SERVO_MAX_POS);
    }
}
