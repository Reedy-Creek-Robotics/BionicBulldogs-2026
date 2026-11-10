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
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

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

    Hive hiveState = Hive.None;


    public void init(Teams team, HardwareMap hwMap, Telemetry telemetry) {
        this.hardwareMap = hwMap;
        this.telemetry = telemetry;
        // set color to teamSide
        this.team = team;
        initCameraValues();
    }

    public void TeamSetRun() {
        if (team == Teams.RED) {
            AprilTagRed();
        } else AprilTagBlue();
    }


    public void initCameraValues() {

        // ======================= CAMERA SETTINGS =======================

        // Stream resolution. The AR0234 sensor is 1920x1200 (16:10).
        // 1280x720 keeps most of the sensor's width and runs well on a Control Hub.
        // NOTE: 4:3 modes like 640x480 may crop the sides, which changes the field of view.
        final int RES_WIDTH = 1280;
        final int RES_HEIGHT = 720;

        // Horizontal field of view of the lens at this resolution, before zoom.
        // ELP lists the H120 lens at roughly 120-126°. Only used if no measured calibration.
        final double LENS_HFOV_DEG = 120.0;

        // Digital zoom. The ELP USBGS1200P01 does NOT support UVC zoom, so this stays at 1.0.
        // Leaving it above 1.0 would scale fx/fy for a zoom that isn't happening and make
        // every reported distance too large. For a tighter view, use a narrower lens instead.
        // (If you ever use a camera that does support zoom: ZOOM_FACTOR is the real
        // magnification, CAMERA_ZOOM_VALUE is the raw UVC number, and they are not the same unit.)
        final double ZOOM_FACTOR = 1.0;
        final int CAMERA_ZOOM_VALUE = 250;

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
        final int DECIMATION_FAR = 1;
        final int DECIMATION_NEAR = 3;
        final double NEAR_RANGE_THRESHOLD_IN = 36.0;
        final double FAR_RANGE_THRESHOLD_IN = 72.0;

        // RC LiveView preview costs CPU. Off by default for speed; set true when you want
        // to watch the image while tuning exposure/gain.
        final boolean ENABLE_LIVE_VIEW = false;

        // ==============================================================

        ExposureControl exposureControl;
        GainControl gainControl;

        int exposureMs = START_EXPOSURE_MS;
        int gain = START_GAIN;
        int minExposureMs = 1;
        int maxExposureMs = 100;
        int minGain = 0;
        int maxGain = 255;

        int currentDecimation = DECIMATION_DEFAULT;
        boolean zoomApplied = false;
        String zoomStatus = "not attempted";

        // For edge detection on gamepad buttons
        boolean lastLB;
        boolean lastRB;
        boolean lastLT;
        boolean lastRT;

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
        telemetry.update();

        // Save CPU resources; can resume streaming when needed.
        visionPortal.stopStreaming();
    }

    public void useCamOn() {
        telemetry.update();

        // Save CPU resources; can resume streaming when needed.
        visionPortal.resumeStreaming();
    }


    public void AprilTagRed() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata.name.equals("RED SCORING")) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    Hive hiveState = Hive.Up;

                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                if (clusterDet.metadata.name.equals("RED AUDIENCE")) {
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    Hive hiveState = Hive.Up;

                } else {
                    telemetry.addLine("hive not detected/hive down");
                    Hive hiveState = Hive.Down;
                }
            }
        }
    }

    public void AprilTagBlue() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata.name.equals("BLUE SCORING")) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    hiveState = Hive.Up;
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                if (clusterDet.metadata.name.equals("BLUE AUDIENCE")) {
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    hiveState = Hive.Up;
                } else {
                    telemetry.addLine("hive not detected");
                    hiveState = Hive.Down;
                }
            }
        }
    }
}
