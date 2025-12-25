package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "Turret", group = "Test")
public class turret extends OpMode {
    public static double target = 0;
    public static boolean face = false;
    public static double x = 0, y = 0;
    private Follower f;
    DcMotor turretmotor = null;
    private PIDFController p, s; // pidf controller for turret
    public static double t = 0,error = 0, power = 0;
    public static double pidfSwitch = 30; // target for turret
    public static double kp = 0.003, kf = 0.0, kd = 0.000, sp = .005, sf = 0, sd = 0.0001;
    public static double rpt = 0.0029919;
    private Limelight3A limelight;
    int tagID = 20;
    double angle;
    private final Pose startPose = new Pose(8, 6.25, Math.toRadians(0));
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        f.startTeleopDrive();
    }
    public void init() {
        turretmotor = hardwareMap.dcMotor.get("turret");
        f = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight3A");
        limelight.setPollRateHz(100);

        f.setStartingPose(new Pose(8, 6.25, Math.toRadians(0)));
        resetTurret();

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }
    public void face(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        setYaw(robotAngleDiff);
    }
    public void faceangle(double angle, double target) {

        double robotAngleDiff = normalizeAngle(angle*(Math.PI * 2D)/180-target);
        setYaw(robotAngleDiff);
    }
    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }
    private void setTurretTarget(double ticks) {
        t = ticks;
    }
    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(radians/rpt);
    }
    public void resetTurret() {
        turretmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setTurretTarget(0);
    }
    public void periodic() {

        p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
        error = getTurretTarget() - getTurret();
        if (Math.abs(error) > pidfSwitch) {
            p.updateError(error);
            p.updateFeedForwardInput(Math.signum(error));
            power = p.run();
        } else {
            s.updateError(error);
            power = s.run();
        }
        turretmotor.setPower(power);
    }
        public double getTurretTarget() {
            return t;
        }

        public double getTurret() {
            return  turretmotor.getCurrentPosition();
        }
    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        // Get the Limelight NetworkTable

        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
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
        }
        f.update();
       // face(new Pose(x, y), f.getPose());
        faceangle(angle,0);
        periodic();
        telemetry.update();
    }
}
