package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tuner (Manual)", group = "Tuning")
public class ServoTunerManual extends LinearOpMode {

    // Change to your configured servo name
    private static final String ballKick = "ballKick";

    // Start value when OpMode begins
    private double pos = 0.50;

    // Step sizes
    private static final double FINE_STEP = 0.001;
    private static final double COARSE_STEP = 0.01;

    private static double clamp01(double x) {
        if (x < 0.0) return 0.0;
        if (x > 1.0) return 1.0;
        return x;
    }

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, ballKick);

        telemetry.addLine("Servo Tuner (Manual) Ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("  dpad up/down  : +/− coarse");
        telemetry.addLine("  x / b         : −/+ fine");
        telemetry.addLine("  y             : snap to 0.50");
        telemetry.addLine("  a             : print value (copy it)");
        telemetry.update();

        waitForStart();

        boolean lastX=false, lastB=false, lastDU=false, lastDD=false, lastY=false, lastA=false;

        while (opModeIsActive()) {
            boolean x  = gamepad1.x;
            boolean b  = gamepad1.b;
            boolean du = gamepad1.dpad_up;
            boolean dd = gamepad1.dpad_down;
            boolean y  = gamepad1.y;
            boolean a  = gamepad1.a;

            // Edge detection: one press = one change
            if (x && !lastX)  pos -= FINE_STEP;
            if (b && !lastB)  pos += FINE_STEP;

            if (du && !lastDU) pos += COARSE_STEP;
            if (dd && !lastDD) pos -= COARSE_STEP;

            if (y && !lastY) pos = 0.50;

            pos = clamp01(pos);
            servo.setPosition(pos);

            // "Print" value for copy/paste
            if (a && !lastA) {
                telemetry.addLine("=== COPY THIS ===");
                telemetry.addLine(String.format("double SERVO_POS = %.5f;", pos));
                telemetry.addLine("=================");
            }

            lastX=x; lastB=b; lastDU=du; lastDD=dd; lastY=y; lastA=a;

            telemetry.addData("Servo", ballKick);
            telemetry.addData("Position (0-1)", "%.5f", pos);
            telemetry.addData("Fine step", FINE_STEP);
            telemetry.addData("Coarse step", COARSE_STEP);
            telemetry.update();
        }
    }
}

