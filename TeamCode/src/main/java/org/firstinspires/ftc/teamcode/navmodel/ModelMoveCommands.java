package org.firstinspires.ftc.teamcode.navmodel;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;

import java.util.function.BooleanSupplier;

public final class ModelMoveCommands {
    private ModelMoveCommands() {
    }

    public interface MoveExtra {
        void queue(ModelMove move);
    }

    public static final class Via implements MoveExtra {
        final double x, y, radius, headingDeg;

        private Via(double x, double y, double radius, double headingDeg) {
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.headingDeg = headingDeg;
        }

        @Override
        public void queue(ModelMove move) { move.via(x, y, radius, headingDeg); }
    }

    public static MoveExtra at(double fraction, Runnable action) { return move -> move.at(fraction, action); }

    public static Via via(double x, double y) { return new Via(x, y, ModelMove.DEFAULT_VIA_RADIUS, Double.NaN); }

    public static Via via(double x, double y, double radius) { return new Via(x, y, radius, Double.NaN); }

    public static Via via(double x, double y, double radius, double headingDeg) { return new Via(x, y, radius, headingDeg); }

    public static CommandBuilder driveTo(ModelMove move, double x, double y, double headingDeg, MoveExtra... vias) {
        return command(move, () -> {
            queue(move, vias);
            move.goTo(x, y, headingDeg);
        });
    }

    public static CommandBuilder driveTo(ModelMove move, double x, double y) {
        return command(move, () -> move.goTo(x, y));
    }

    public static CommandBuilder driveTo(ModelMove move, double x, double y, MoveExtra first, MoveExtra... more) {
        return command(move, () -> {
            queue(move, first);
            queue(move, more);
            move.goTo(x, y);
        });
    }

    public static CommandBuilder sleepAndHold(ModelMove move, long ms) {
        return command(move, () -> move.hold(ms), () -> !move.isHolding());
    }

    public static CommandBuilder instant(Runnable action) {
        return new CommandBuilder().setStart(action).setDone(() -> true);
    }

    private static void queue(ModelMove move, MoveExtra... vias) {
        for (MoveExtra v : vias) v.queue(move);
    }

    private static CommandBuilder command(ModelMove move, Runnable start) {
        return command(move, start, () -> !move.isBusy());
    }

    private static CommandBuilder command(ModelMove move, Runnable start, BooleanSupplier done) {
        return new CommandBuilder()
                .requiring(move)
                .setStart(start)
                .setExecute(move::update)
                .setDone(done)
                .setEnd(end -> { if (end != EndCondition.NATURALLY) move.stop(); });
    }
}
