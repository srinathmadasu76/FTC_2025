package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name = "Sensor: Limelight3A", group = "Test")
public class SensorLimelight3A extends OpMode {

    private Limelight3A limelight;
    DcMotor ShooterMotor = null;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    public static double bp = 0.01, bd = 0.0, bf = 0.0, sp = 0.01, sd = 0.0001, sf = 0.0;
    double targetvel = 1100;
    double pSwitch = 50;
    Servo hood = null;
    private PIDFController b, s;
    double angle;
    double power_rotation = 0.2;
    double angle_positive = 5;
    double angle_negative = -3;
    int tagID = 20;

    @Override
    public void init() {
        FrontLeft = hardwareMap.dcMotor.get("FL")   ;
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        ShooterMotor = hardwareMap.dcMotor.get("shooter");
        hood = hardwareMap.get(Servo.class,("hood"));
        hood.setPosition(0.24);
        limelight = hardwareMap.get(Limelight3A.class, "limelight3A");
        limelight.setPollRateHz(100);
        // telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
    }
    public void start()
    {
        limelight.start();
    }
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.

         */
       public void loop()
       {
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        //waitForStart();

     //   while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            // Get the Limelight NetworkTable

            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }


                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    if (gamepad1.dpad_down) {
                        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();

                        LLResultTypes.FiducialResult target = null;
                        for (LLResultTypes.FiducialResult i : r) {
                            if (i != null && i.getFiducialId() == tagID) {
                                target = i;
                                break;
                            }
                        }

                        if (target != null) {
                            angle = target.getTargetXDegrees();
                            telemetry.addData("angle", angle);
                            //telemetry.addData("robotheading:", follower::getHeading);
                        }
                        while(angle>angle_positive)
                        {
                            FrontRight.setPower(-power_rotation);
                            BackLeft.setPower(power_rotation);
                            FrontLeft.setPower(power_rotation);
                            BackRight.setPower(-power_rotation);

                            r = limelight.getLatestResult().getFiducialResults();

                            target = null;
                            for (LLResultTypes.FiducialResult i : r) {
                                if (i != null && i.getFiducialId() == tagID) {
                                    target = i;
                                    break;
                                }
                            }

                            if (target != null) {
                                angle = target.getTargetXDegrees();
                                telemetry.addData("angle", angle);
                                //telemetry.addData("robotheading:", follower::getHeading);
                            }
                        }
                        while(angle<angle_negative)
                        {
                            FrontRight.setPower(power_rotation);
                            BackLeft.setPower(-power_rotation);
                            FrontLeft.setPower(-power_rotation);
                            BackRight.setPower(power_rotation);

                            r = limelight.getLatestResult().getFiducialResults();

                            target = null;
                            for (LLResultTypes.FiducialResult i : r) {
                                if (i != null && i.getFiducialId() == tagID) {
                                    target = i;
                                    break;
                                }
                            }

                            if (target != null) {
                                angle = target.getTargetXDegrees();
                                telemetry.addData("angle", angle);
                                //telemetry.addData("robotheading:", follower::getHeading);
                            }
                        }
                    }

                    List<LLResultTypes.FiducialResult> r =limelight.getLatestResult().getFiducialResults();



                    LLResultTypes.FiducialResult target = null;
                    for (LLResultTypes.FiducialResult i: r) {
                        if (i != null && i.getFiducialId() ==  tagID) {
                            target = i;
                            break;
                        }
                    }

                    if (target != null) {
                        double x = (target.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 0; // right/left from tag
                        double z = (target.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 8; // forward/back from tag

                        Vector e = new Vector();
                        e.setOrthogonalComponents(x, z);
                        telemetry.addData("Distance", e.getMagnitude());
                    }
                    double currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
                    b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
                    s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
                    b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
                    s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

                    if (Math.abs(targetvel - currentvel) < pSwitch) {
                        s.updateError(targetvel - currentvel);
                        ShooterMotor.setPower(s.run());
                    } else {
                        b.updateError(targetvel - currentvel);
                        telemetry.addData("power1", b.run());
                        ShooterMotor.setPower(b.run());
                    }
                    currentvel = ((DcMotorEx)ShooterMotor).getVelocity();
                    telemetry.addData("velocity1", currentvel);

                    // Access detector results
                 /*   List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }*/

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
       // limelight.stop();
    }

