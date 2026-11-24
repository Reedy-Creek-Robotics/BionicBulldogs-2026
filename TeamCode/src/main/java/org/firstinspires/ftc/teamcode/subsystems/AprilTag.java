package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class AprilTag {
    public static final boolean USE_WEBCAM = true;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public enum Teams {
        RED,
        BLUE,
        NONE,

    }

    Teams team = Teams.NONE;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public enum Hive {
        Up,
        Down,
        None,

    }

    public Hive hiveState;


    public void init(Teams team, HardwareMap hwMap, Telemetry telemetry) {
        hiveState = Hive.None;
        this.hardwareMap = hwMap;
        this.telemetry = telemetry;
        this.team = team;
        initCameraValues();
    }

    public void initCameraValues() {

        // ======================= CAMERA SETTINGS =======================

        // Stream resolution. The AR0234 sensor is 1920x1200 (16:10).
        // 1280x720 keeps most of the sensor's width and runs well on a Control Hub.
        // NOTE: 4:3 modes like 640x480 may crop the sides, which changes the field of view.
        final int RES_WIDTH = 1280;
        final int RES_HEIGHT = 720;





        // ==================== MEASURED CALIBRATION ======================
        // Once you calibrate (3DF Zephyr, with the zoom applied, at RES_WIDTH x RES_HEIGHT),
        // set this to true and paste your values. This beats any estimate.
        final boolean USE_MEASURED_CALIBRATION = false;
        final double MEASURED_FX = 0;
        final double MEASURED_FY = 0;
        final double MEASURED_CX = 0;
        final double MEASURED_CY = 0;

        // ==================== EXPOSURE / GAIN ======================
        // Start short (~5 ms) and raise gain to compensate for the darker image.
        final int START_EXPOSURE_MS = 5;
        final int START_GAIN = 150;

        // ==================== DECIMATION ======================
        // Lower decimation = longer range but slower.
        // DEFAULT is used while searching (no tag seen). Decimation 1 is only used when a tag
        // is genuinely far away, since running it on every frame is what dropped FPS to ~8.
        final int DECIMATION_DEFAULT = 2;



        // RC LiveView preview costs CPU. Off by default for speed; set true when you want
        // to watch the image while tuning exposure/gain.
        final boolean ENABLE_LIVE_VIEW = false;

        // ==============================================================

        ExposureControl exposureControl;
        GainControl gainControl;

        int exposureMs = START_EXPOSURE_MS;
        int gain = START_GAIN;


        int currentDecimation = DECIMATION_DEFAULT;


        //April tags init
        double fx, fy, cx, cy;

        fx = MEASURED_FX;
        fy = MEASURED_FY;
        cx = MEASURED_CX;
        cy = MEASURED_CY;


        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(fx, fy, cx, cy)
                // Cheaper annotations: outline only.
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .build();

        aprilTag.setDecimation(currentDecimation);


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(RES_WIDTH, RES_HEIGHT))
                // MJPEG: much less USB bandwidth than YUY2, so higher res at higher FPS.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(ENABLE_LIVE_VIEW)
                .setAutoStopLiveView(false)
                .addProcessor(aprilTag)
                .build();

    }

    public void useCamOff() {
        // Save CPU resources; can resume streaming when needed.
        visionPortal.stopStreaming();
    }

    public void useCamOn() {
        telemetry.update();
        // Save CPU resources; can resume streaming when needed.
        visionPortal.resumeStreaming();
    }

    public void hiveGoal(double y ){
        if (team == team.RED){
            if (y >= 70.75){
                AprilTagRedScoring();
            }else{
                AprilTagRedAudience();
            }
        }else{
            if (y >= 70.75){
                AprilTagBlueScoring();
            }else{
                AprilTagBlueAudience();
            }
        }
    }



    public Hive AprilTagRedAudience() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata.name.equals("RED AUDIENCE")) {
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    hiveState = Hive.Up;
                    return hiveState;

                }
            }
        }
        telemetry.addLine("hive not detected/hive down");
        hiveState = Hive.Down;
        return hiveState;
    }

    public Hive AprilTagRedScoring() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata.name.equals("RED SCORING")) {
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    hiveState = Hive.Up;
                    return hiveState;
                }
            }
        }
        telemetry.addLine("hive not detected/hive down");
        hiveState = Hive.Down;
        return hiveState;
    }

    public Hive AprilTagBlueScoring() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata.name.equals("BLUE SCORING")) {
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    hiveState = Hive.Up;
                    return hiveState;
                }
            }
        }
        telemetry.addLine("hive not detected/hive down");
        hiveState = Hive.Down;
        return hiveState;
    }


    public Hive AprilTagBlueAudience() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata.name.equals("BLUE AUDIENCE")) {
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    hiveState = Hive.Up;
                    return hiveState;
                }
            }
        }
        telemetry.addLine("hive not detected/hive down");
        hiveState = Hive.Down;
        return hiveState;
    }
}
