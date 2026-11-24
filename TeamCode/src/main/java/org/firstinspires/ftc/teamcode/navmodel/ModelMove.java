package org.firstinspires.ftc.teamcode.navmodel;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.function.BooleanSupplier;

public class ModelMove {
    public static final long SLOW_LOOP_NS = 100_000_000L;
    public static final int MAX_VIA = 8;
    public static final int MAX_AT = 24;
    public static final double DEFAULT_VIA_RADIUS = 10.0;
    public static final double MIN_VIA_RADIUS = ModelDrive.POS_TOL;
    public static final double HOLD_POS_TOL = 3.0, HOLD_HEAD_TOL = 0.15;
    private static final int VOLTAGE_EVERY = 5;
    private static final String[] NAMES = {NavHardware.FL, NavHardware.FR, NavHardware.BL, NavHardware.BR};

    public final ModelDrive drive;
    public final GoBildaPinpointDriver pinpoint;
    private final OpMode opMode;
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final VoltageSensor voltage;
    private final double[] wheels = new double[4];
    private BooleanSupplier enable;
    private String refused;
    private long nextTickNs, lastTickNs;
    private int tick, slowLoops;
    private boolean powered, poseOk;
    private double x, y, h, vx, vy, w, vbat = 12.0;
    private final double[] qx = new double[MAX_VIA], qy = new double[MAX_VIA], qr = new double[MAX_VIA], qh = new double[MAX_VIA];
    private int queued;
    private String queueError;
    private final double[] qaf = new double[MAX_AT], af = new double[MAX_AT];
    private final Runnable[] qar = new Runnable[MAX_AT], ar = new Runnable[MAX_AT];
    private final boolean[] aran = new boolean[MAX_AT];
    private int queuedActs, acts;
    private double pathLen = Double.NaN, progress = Double.NaN;
    private final double[] lx = new double[MAX_VIA + 1], ly = new double[MAX_VIA + 1], lr = new double[MAX_VIA];
    private final double[] lh = new double[MAX_VIA + 1];
    private int legs, leg;
    private boolean holding, held;
    private long holdEndNs;
    private double hx, hy, hh;
    private int holdReturns;

    public ModelMove(OpMode opMode, double startX, double startY, double startHeadingDeg) {
        this.opMode = opMode;
        HardwareMap hw = opMode.hardwareMap;
        motors[0] = NavHardware.driveMotor(hw, NavHardware.FL);
        motors[1] = NavHardware.driveMotor(hw, NavHardware.FR);
        motors[2] = NavHardware.driveMotor(hw, NavHardware.BL);
        motors[3] = NavHardware.driveMotor(hw, NavHardware.BR);
        VoltageSensor v = null;
        for (VoltageSensor s : hw.voltageSensor) { v = s; break; }
        voltage = v;
        if (voltage != null && voltage.getVoltage() > 5.0) vbat = voltage.getVoltage();
        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");
        NavHardware.configurePinpoint(pinpoint);
        drive = new ModelDrive(startX > ModelDrive.LINE_X);
        setPose(startX, startY, startHeadingDeg);
    }

    public boolean driveTo(double x, double y, double headingDeg) { return driveTo(x, y, headingDeg, null); }

    public boolean driveTo(double x, double y, double headingDeg, Runnable eachLoop) {
        linear();
        return goTo(x, y, headingDeg) && waitForMove(eachLoop);
    }

    public boolean driveTo(double x, double y) { return driveTo(x, y, null); }

    public boolean driveTo(double x, double y, Runnable eachLoop) {
        linear();
        return goTo(x, y) && waitForMove(eachLoop);
    }

    public ModelMove via(double x, double y) { return via(x, y, DEFAULT_VIA_RADIUS); }

    public ModelMove via(double x, double y, double radius) { return via(x, y, radius, Double.NaN); }

    public ModelMove via(double x, double y, double radius, double headingDeg) {
        if (queueError != null) return this;
        if (queued >= MAX_VIA) queueError = "more than " + MAX_VIA + " intermediate points";
        else if (!(Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(radius)))
            queueError = "intermediate point " + (queued + 1) + ": not a number";
        else {
            qx[queued] = x;
            qy[queued] = y;
            qh[queued] = Double.isNaN(headingDeg) ? Double.NaN : Math.toRadians(headingDeg);
            qr[queued++] = Math.max(MIN_VIA_RADIUS, radius);
        }
        return this;
    }

    public ModelMove at(double fraction, Runnable action) {
        if (queueError != null) return this;
        if (queuedActs >= MAX_AT) queueError = "more than " + MAX_AT + " actions";
        else if (action == null || !Double.isFinite(fraction)) queueError = "action " + (queuedActs + 1) + ": none, or not a number";
        else {
            qaf[queuedActs] = Math.max(0.0, Math.min(1.0, fraction));
            qar[queuedActs++] = action;
        }
        return this;
    }

    public boolean sleepAndHold(long ms, Runnable eachLoop) {
        linear();
        hold(ms);
        runToEnd(eachLoop);
        return held;
    }

    public boolean sleepAndHold(long ms) { return sleepAndHold(ms, null); }

    public void hold(long ms) {
        legs = leg = 0;
        queued = 0;
        queueError = null;
        dropActions();
        queuedActs = 0;
        if (drive.busy()) drive.stop();
        readPose();
        ModelDrive.Status s = drive.status();
        double gx = drive.goalX(), gy = drive.goalY(), gh = drive.goalHeading();
        if (!poseOk) hx = hy = hh = Double.NaN;
        else if (s != ModelDrive.Status.IDLE && s != ModelDrive.Status.UNREACHABLE && !Double.isNaN(gh) && !off(gx, gy, gh)) {
            hx = gx;
            hy = gy;
            hh = gh;
        } else {
            hx = x;
            hy = y;
            hh = h;
        }
        holding = true;
        held = false;
        holdReturns = 0;
        holdEndNs = System.nanoTime() + Math.max(0L, ms) * 1_000_000L;
        refused = null;
        nextTickNs = 0;
        lastTickNs = 0;
        for (int i = 0; i < 4; i++) NavHardware.ensureDriveSetup(motors[i], NAMES[i]);
        powered = true;
        brake();
    }

    public boolean goTo(double x, double y, double headingDeg) { return begin(x, y, Math.toRadians(headingDeg), false); }

    public boolean goTo(double x, double y) { return begin(x, y, Double.NaN, true); }

    private boolean begin(double x, double y, double heading, boolean keep) {
        holding = false;
        boolean was = drive.busy();
        int n = queued;
        String err = queueError;
        queued = 0;
        queueError = null;
        legs = leg = 0;
        dropActions();
        acts = err == null ? queuedActs : 0;
        for (int i = 0; i < acts; i++) {
            af[i] = qaf[i];
            ar[i] = qar[i];
            aran[i] = false;
        }
        queuedActs = 0;
        pathLen = Double.NaN;
        lx[0] = x;
        ly[0] = y;
        if (err != null) {
            drive.goTo(Double.NaN, Double.NaN, Double.NaN);
            return started(was, false, err);
        }
        if (n == 0) {
            boolean ok = keep ? drive.goTo(x, y) : drive.goTo(x, y, heading);
            return started(was, ok, ok ? null : keep ? "not a number" : drive.unreachableReason(x, y, heading));
        }
        if (keep) {
            readPose();
            heading = h;
        }
        for (int i = 0; i <= n; i++) {
            double px = i < n ? qx[i] : x, py = i < n ? qy[i] : y;
            double ph = i < n && !Double.isNaN(qh[i]) ? qh[i] : heading;
            String why = drive.unreachableReason(px, py, ph);
            if (why != null) {
                drive.goTo(px, py, ph);
                return started(was, false, i < n ? "intermediate point " + (i + 1) + ": " + why : why);
            }
        }
        for (int i = 0; i < n; i++) {
            lx[i] = qx[i];
            ly[i] = qy[i];
            lr[i] = qr[i];
            lh[i] = Double.isNaN(qh[i]) ? heading : qh[i];
        }
        lx[n] = x;
        ly[n] = y;
        lh[n] = heading;
        legs = n;
        return started(was, drive.goTo(lx[0], ly[0], lh[0]), null);
    }

    private boolean started(boolean wasBusy, boolean ok, String reason) {
        refused = reason;
        if (!ok) dropActions();
        if (!wasBusy) {
            nextTickNs = 0;
            lastTickNs = 0;
            if (ok) for (int i = 0; i < 4; i++) NavHardware.ensureDriveSetup(motors[i], NAMES[i]);
        }
        if (!ok) brake();
        return ok;
    }

    public boolean update() {
        long now = System.nanoTime();
        if (nextTickNs != 0 && now < nextTickNs) return running();
        boolean slow = lastTickNs != 0 && now - lastTickNs > SLOW_LOOP_NS;
        long dt = ModelDrive.CONTROL_DT_NS;
        nextTickNs = (nextTickNs == 0 || now - nextTickNs >= dt) ? now + dt : nextTickNs + dt;
        lastTickNs = now;
        readPose();
        if (voltage != null && ++tick % VOLTAGE_EVERY == 0) {
            double v = voltage.getVoltage();
            if (v > 5.0) vbat = v;
        }
        if (holding) {
            if (now - holdEndNs >= 0) {
                holding = false;
                held = poseOk && !Double.isNaN(hx) && !off(hx, hy, hh);
                if (drive.busy()) drive.stop();
                brake();
                return false;
            }
            if (!drive.busy()) {
                if (!off(hx, hy, hh)) {
                    brake();
                    return true;
                }
                if (!drive.goTo(hx, hy, hh)) {
                    hx = hy = hh = Double.NaN;
                    brake();
                    return true;
                }
                holdReturns++;
            }
        }
        if (!drive.busy()) {
            brake();
            return false;
        }
        if (slow) slowLoops++;
        if (slow || (enable != null && !enable.getAsBoolean())) {
            drive.pause();
            brake();
            return true;
        }
        if (leg < legs && poseOk && Math.hypot(x - lx[leg], y - ly[leg]) < lr[leg]) {
            leg++;
            drive.goTo(lx[leg], ly[leg], lh[leg]);
        }
        if (acts > 0 && poseOk) runActions(measureProgress());
        double nan = Double.NaN;
        double[] c = poseOk ? drive.update(x, y, h, vx, vy, w) : drive.update(nan, nan, nan, nan, nan, nan);
        if (!drive.busy()) {
            brake();
            runActions(Double.POSITIVE_INFINITY);
            return holding;
        }
        ModelDrive.wheelPowers(c, vbat, wheels);
        for (int i = 0; i < 4; i++) motors[i].setPower(wheels[i]);
        powered = true;
        return true;
    }

    public boolean waitForMove(Runnable eachLoop) {
        runToEnd(eachLoop);
        return drive.atGoal();
    }

    private void runToEnd(Runnable eachLoop) {
        LinearOpMode op = linear();
        while (running()) {
            if (!op.opModeIsActive()) {
                stop();
                break;
            }
            update();
            if (eachLoop != null) eachLoop.run();
            long waitMs = (nextTickNs - System.nanoTime()) / 1_000_000L;
            if (waitMs > 0) op.sleep(waitMs); else op.idle();
        }
    }

    public void stop() {
        holding = false;
        drive.stop();
        legs = leg = 0;
        dropActions();
        powered = true;
        brake();
    }

    public void setEnable(BooleanSupplier enable) { this.enable = enable; }

    public void setTimeout(double seconds) { drive.setTimeout(seconds); }

    public void setMaxPower(double power) { drive.setMaxPower(power); }

    public void setBlueAlliance(boolean blue) {
        drive.setBlueAlliance(blue);
        holding = false;
        legs = leg = 0;
        brake();
    }

    public void setPose(double x, double y, double headingDeg) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg));
        readPose();
    }

    public boolean isBusy() { return drive.busy(); }

    public boolean isHolding() { return holding; }

    public int holdReturns() { return holdReturns; }

    public boolean atGoal() { return drive.atGoal(); }

    public ModelDrive.Status status() { return drive.status(); }

    public int viaCount() { return legs; }

    public int viaPassed() { return leg; }

    public String whyRefused() { return refused; }

    public double x() { return x; }

    public double y() { return y; }

    public double headingDeg() { return Math.toDegrees(h); }

    public boolean poseOk() { return poseOk; }

    public double batteryVolts() { return vbat; }

    public int slowLoops() { return slowLoops; }

    public double progress() { return progress; }

    public void addTelemetry() {
        Telemetry t = opMode.telemetry;
        ModelDrive.Status s = drive.status();
        if (holding) t.addData("Hold", Double.isNaN(hx) ? "no pose: braking" : String.format("(%.1f, %.1f) %.0f deg  %.1f s left, %d returns",
                hx, hy, Math.toDegrees(hh), Math.max(0L, holdEndNs - System.nanoTime()) / 1e9, holdReturns));
        t.addData("Move", s == ModelDrive.Status.UNREACHABLE && refused != null ? "UNREACHABLE: " + refused
                : String.format("%s  %.1f s", s, drive.elapsedS()));
        if (s == ModelDrive.Status.IDLE) t.addData("Goal", "none");
        else t.addData(leg < legs ? "Via" : "Goal", "(%.1f, %.1f) %.0f deg%s", drive.goalX(), drive.goalY(),
                Math.toDegrees(drive.goalHeading()), legs > 0 ? String.format("  (point %d of %d)", leg + 1, legs + 1) : "");
        t.addData("Pose", "(%.1f, %.1f) %.0f deg%s", x, y, Math.toDegrees(h), poseOk ? "" : "  PINPOINT NOT READY");
        t.addData("To go", "%.1f in, %.1f deg", drive.distanceToGoal(), Math.toDegrees(drive.headingErrorToGoal()));
        t.addData("Alliance / battery / slow loops", "%s / %.2f V / %d", drive.isBlue() ? "BLUE" : "RED", vbat, slowLoops);
    }

    private double measureProgress() {
        double rest = Math.hypot(lx[leg] - x, ly[leg] - y);
        for (int i = leg; i < legs; i++) rest += Math.hypot(lx[i + 1] - lx[i], ly[i + 1] - ly[i]);
        if (Double.isNaN(pathLen)) pathLen = rest;
        progress = pathLen > 1e-6 ? 1.0 - rest / pathLen : 1.0;
        return progress;
    }

    private void runActions(double p) {
        for (int i = 0; i < acts; i++) {
            if (!aran[i] && af[i] <= p) {
                aran[i] = true;
                ar[i].run();
            }
        }
    }

    private void dropActions() {
        for (int i = 0; i < acts; i++) ar[i] = null;
        acts = 0;
        progress = Double.NaN;
    }

    private boolean running() { return holding || drive.busy(); }

    private boolean off(double px, double py, double ph) {
        if (!poseOk) return false;
        return Math.hypot(x - px, y - py) > HOLD_POS_TOL || Math.abs(Math.atan2(Math.sin(h - ph), Math.cos(h - ph))) > HOLD_HEAD_TOL;
    }

    private void readPose() {
        pinpoint.update();
        poseOk = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        h = pinpoint.getHeading(AngleUnit.RADIANS);
        vx = pinpoint.getVelX(DistanceUnit.INCH);
        vy = pinpoint.getVelY(DistanceUnit.INCH);
        w = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    private void brake() {
        if (!powered) return;
        for (DcMotorEx m : motors) m.setPower(0);
        powered = false;
    }

    private LinearOpMode linear() {
        if (!(opMode instanceof LinearOpMode))
            throw new IllegalStateException("driveTo blocks: use it in a LinearOpMode, or goTo + update() in an OpMode loop");
        return (LinearOpMode) opMode;
    }
}
