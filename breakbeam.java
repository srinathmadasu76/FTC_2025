package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "BeamBreakSampleOpMode", group = "Test")
public class breakbeam extends LinearOpMode {

    private DigitalChannel beamBreakSensor;

    @Override
    public void runOpMode() {
        // Map the sensor to the name configured on the Robot Controller
        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "breakbeam");

        // Set the digital channel to input mode (required for DigitalChannel use)
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Most beam-break sensors read:
            // HIGH (true) = beam clear
            // LOW  (false) = beam broken
            boolean isObjectDetected = !beamBreakSensor.getState();

            telemetry.addData("Beam Raw State", beamBreakSensor.getState() ? "HIGH (clear)" : "LOW (broken)");
            telemetry.addData("Beam Status", isObjectDetected
                    ? "BROKEN - Object Detected"
                    : "CLEAR - No Object");

            // Example action:
            // if (isObjectDetected) intakeMotor.setPower(0);

            telemetry.update();
            idle(); // good practice to yield CPU
        }
    }
}
