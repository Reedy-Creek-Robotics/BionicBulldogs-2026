package org.firstinspires.ftc.teamcode.navmodel;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

public class NavHardware {

    public static final String FL = "front_left_motor", FR = "front_right_motor",
            BL = "back_left_motor", BR = "back_right_motor";
    public static final double GATE_OPEN = 0.0, GATE_CLOSED = 0.3;
    public static final double NOMINAL_V = 12.0;
    public static final int VOLTAGE_EVERY = 5, CURRENT_EVERY = 10;

    public static final String[] START_NAMES = {
            "Back wall centre, facing +Y", "Field centre, facing +Y", "Field centre, facing +X",
            "Back-left (24, 24), facing +Y", "Back-right (120, 24), facing +Y"};
    public static final double[][] START_POSES = {
            {72.0, 10.25, Math.PI / 2}, {72.0, 72.0, Math.PI / 2}, {72.0, 72.0, 0.0},
            {24.0, 24.0, Math.PI / 2}, {120.0, 24.0, Math.PI / 2}};

    public static final class Snapshot {
        public long nanos;
        public final double[] enc = new double[4], vel = new double[4];
        public double ppX = Double.NaN, ppY = Double.NaN, ppH = Double.NaN;
        public double ppVx = Double.NaN, ppVy = Double.NaN, ppVh = Double.NaN;
        public double ppEncX = Double.NaN, ppEncY = Double.NaN;
        public boolean ppReady;
        public double vbat = Double.NaN;
        public double intakeCurrent = Double.NaN;
        public double panPos = Double.NaN;
        public boolean llNew;
        public double llLatencyMs = Double.NaN, llStalenessMs = Double.NaN;
        public double[] llPy;
    }

    public final Snapshot s = new Snapshot();
    public final double[] cmdWheel = new double[4];
    public double cmdIntake = Double.NaN, cmdGate = Double.NaN, cmdPan = Double.NaN, cmdShooter = Double.NaN;

    public DcMotorEx fl, fr, bl, br;
    public DcMotorEx intake, shooter;
    public Servo gate;
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;
    public VoltageSensor voltage;
    public final List<String> missing = new ArrayList<>();

    private final DcMotorEx[] drive = new DcMotorEx[4];
    private final List<LynxModule> hubs = new ArrayList<>();
    private final List<CRServo> crServos = new ArrayList<>();
    private boolean voltageComp;
    private boolean brakeMode = true;
    private int loopCount;
    public int currentEvery = CURRENT_EVERY;
    private double lastLlStamp = Double.NaN;
    private long lastLlNanos = -1;

    public void init(HardwareMap hw, double startX, double startY, double startH) {
        for (LynxModule m : hw.getAll(LynxModule.class)) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hubs.add(m);
        }

        fl = driveMotor(hw, FL);
        fr = driveMotor(hw, FR);
        bl = driveMotor(hw, BL);
        br = driveMotor(hw, BR);
        drive[0] = fl; drive[1] = fr; drive[2] = bl; drive[3] = br;
        brakeMode = true;

        intake = hw.tryGet(DcMotorEx.class, "intakeMotor");
        if (intake != null) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setPower(0);
            cmdIntake = 0;
        } else missing.add("intakeMotor");

        gate = hw.tryGet(Servo.class, "GateServo");
        if (gate != null) {
            gate.setPosition(GATE_CLOSED);
            cmdGate = GATE_CLOSED;
        } else missing.add("GateServo");

        shooter = hw.tryGet(DcMotorEx.class, "shooterDriveMotor");
        if (shooter != null) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setPower(0);
            cmdShooter = 0;
        } else missing.add("shooterDriveMotor");

        crServos.addAll(hw.getAll(CRServo.class));

        for (VoltageSensor v : hw.voltageSensor) { voltage = v; break; }
        if (voltage == null) missing.add("voltage sensor");
        else s.vbat = voltage.getVoltage();

        limelight = hw.tryGet(Limelight3A.class, "limelight");
        if (limelight != null) limelight.start(); else missing.add("limelight");

        pinpoint = hw.tryGet(GoBildaPinpointDriver.class, "pinpoint");
        if (pinpoint != null) {
            configurePinpoint(pinpoint);
            setPose(startX, startY, startH);
        } else missing.add("pinpoint");
        readSensors();
    }

    public static void configurePinpoint(GoBildaPinpointDriver pinpoint) {
        pinpoint.setOffsets(3.3483159823680486, -5.152386642816499, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.recalibrateIMU();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        pinpoint.update();
    }

    public static DcMotorEx driveMotor(HardwareMap hw, String name) {
        DcMotorEx m = hw.get(DcMotorEx.class, name);
        m.setDirection(driveDirection(name));
        m.setPower(0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    public static void ensureDriveSetup(DcMotorEx m, String name) {
        DcMotorSimple.Direction d = driveDirection(name);
        if (m.getDirection() != d) m.setDirection(d);
        if (m.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (m.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static DcMotorSimple.Direction driveDirection(String name) {
        return name.equals(FL) || name.equals(BL) ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    public void setPose(double x, double y, double h) {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, h));
        pinpoint.update();
    }

    public void readSensors() {
        for (int i = 0; i < hubs.size(); i++) hubs.get(i).clearBulkCache();
        loopCount++;
        s.nanos = System.nanoTime();
        for (int i = 0; i < 4; i++) {
            s.enc[i] = drive[i].getCurrentPosition();
            s.vel[i] = drive[i].getVelocity();
        }
        if (pinpoint != null) {
            pinpoint.update();
            s.ppReady = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
            s.ppX = pinpoint.getPosX(DistanceUnit.INCH);
            s.ppY = pinpoint.getPosY(DistanceUnit.INCH);
            s.ppH = pinpoint.getHeading(AngleUnit.RADIANS);
            s.ppVx = pinpoint.getVelX(DistanceUnit.INCH);
            s.ppVy = pinpoint.getVelY(DistanceUnit.INCH);
            s.ppVh = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            s.ppEncX = pinpoint.getEncoderX();
            s.ppEncY = pinpoint.getEncoderY();
        }
        if (voltage != null && loopCount % VOLTAGE_EVERY == 0) s.vbat = voltage.getVoltage();
        s.intakeCurrent = (intake != null && loopCount % currentEvery == 0)
                ? intake.getCurrent(CurrentUnit.AMPS) : Double.NaN;
        pollLimelight();
    }

    private void pollLimelight() {
        s.llNew = false;
        s.llPy = null;
        s.llLatencyMs = Double.NaN;
        s.llStalenessMs = Double.NaN;
        if (limelight == null) return;
        LLResult r = limelight.getLatestResult();
        if (r == null) return;
        double stamp = r.getTimestamp();
        long nanos = r.getControlHubTimeStampNanos();
        if (stamp == lastLlStamp && nanos == lastLlNanos) return;
        lastLlStamp = stamp;
        lastLlNanos = nanos;
        s.llNew = true;
        s.llLatencyMs = r.getCaptureLatency() + r.getTargetingLatency() + r.getParseLatency();
        s.llStalenessMs = r.getStaleness();

        try {
            JSONArray a = new JSONObject(r.toString()).optJSONArray("PythonOut");
            if (a != null) {
                double[] py = new double[a.length()];
                for (int i = 0; i < py.length; i++) py[i] = a.optDouble(i, Double.NaN);
                s.llPy = py;
            }
        } catch (Exception e) {
            s.llPy = null;
        }
    }

    public void setVoltageCompensation(boolean on) { voltageComp = on; }

    public void setDrive(double vx, double vy, double omega) {
        if (voltageComp && s.vbat > 5.0) {
            double k = NOMINAL_V / s.vbat;
            vx *= k; vy *= k; omega *= k;
        }
        double pfl = vx - vy - omega, pfr = vx + vy + omega, pbl = vx + vy - omega, pbr = vx - vy + omega;
        double m = Math.max(1.0, Math.max(Math.max(Math.abs(pfl), Math.abs(pfr)), Math.max(Math.abs(pbl), Math.abs(pbr))));
        cmdWheel[0] = pfl / m; cmdWheel[1] = pfr / m; cmdWheel[2] = pbl / m; cmdWheel[3] = pbr / m;
        for (int i = 0; i < 4; i++) drive[i].setPower(cmdWheel[i]);
    }

    public void setZeroPowerBehavior(boolean brake) {
        if (brake == brakeMode) return;
        brakeMode = brake;
        DcMotor.ZeroPowerBehavior b = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        for (DcMotorEx m : drive) m.setZeroPowerBehavior(b);
    }

    public boolean isBrake() { return brakeMode; }

    public void setIntake(double p) {
        if (intake == null || p == cmdIntake) return;
        intake.setPower(p);
        cmdIntake = p;
    }

    public void setGate(boolean open) {
        if (gate == null) return;
        double pos = open ? GATE_OPEN : GATE_CLOSED;
        if (pos == cmdGate) return;
        gate.setPosition(pos);
        cmdGate = pos;
    }

    public void setShooterVelocity(double tps) {
        if (shooter == null || tps == cmdShooter) return;
        if (tps == 0) shooter.setPower(0); else shooter.setVelocity(tps);
        cmdShooter = tps;
    }

    public void stopAll() {
        setDrive(0, 0, 0);
        setZeroPowerBehavior(true);
        setIntake(0);
        setShooterVelocity(0);
        setGate(false);
        for (int i = 0; i < crServos.size(); i++) crServos.get(i).setPower(0);
    }

    public void shutdown() {
        stopAll();
        if (limelight != null) limelight.stop();
    }

    public static int buttonMask(Gamepad g) {
        if (g == null) return 0;
        int m = 0;
        if (g.a) m |= 1;
        if (g.b) m |= 2;
        if (g.x) m |= 4;
        if (g.y) m |= 8;
        if (g.left_bumper) m |= 16;
        if (g.right_bumper) m |= 32;
        if (g.dpad_up) m |= 64;
        if (g.dpad_down) m |= 128;
        if (g.dpad_left) m |= 256;
        if (g.dpad_right) m |= 512;
        if (g.start) m |= 1024;
        if (g.back) m |= 2048;
        if (g.left_stick_button) m |= 4096;
        if (g.right_stick_button) m |= 8192;
        if (g.guide) m |= 16384;
        if (g.touchpad) m |= 32768;
        return m;
    }

    public static boolean anyInput(Gamepad g, boolean ignoreA, double deadband) {
        if (g == null) return false;
        if (Math.abs(g.left_stick_x) > deadband || Math.abs(g.left_stick_y) > deadband
                || Math.abs(g.right_stick_x) > deadband || Math.abs(g.right_stick_y) > deadband
                || g.left_trigger > deadband || g.right_trigger > deadband) return true;
        int m = buttonMask(g);
        if (ignoreA) m &= ~1;
        return m != 0;
    }
}
