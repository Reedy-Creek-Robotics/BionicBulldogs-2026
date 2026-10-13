package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints.HivePositions.spaceFromCenterToGoalsX;
import static org.firstinspires.ftc.teamcode.robot.BotDimensions.botYSize;

import com.pedropathing.geometry.Pose;

public class FieldPoints {
    public static class GeneralConstants {
        public static final double tileSize = 144 / 6;
        public static final double tileSeam = 0.75;
    }
    public static class HivePositions {
        public static final double spaceFromCenterToGoalsX = 25.5 / 2; // space between hives / 2
        public final double spaceFromCenterToGoalsY = Math.cos(30) * 15.44;
        public final Pose redUp = new Pose((72 - spaceFromCenterToGoalsX), (72 - spaceFromCenterToGoalsY));
        public final Pose redDown = new Pose((72 - spaceFromCenterToGoalsX), (72 + spaceFromCenterToGoalsY));
        public final Pose blueUp = new Pose((72 + spaceFromCenterToGoalsX), (72 - spaceFromCenterToGoalsY));
        public final Pose blueDown = new Pose((72 + spaceFromCenterToGoalsX), (72 + spaceFromCenterToGoalsY));
    }

    public static class StartingPositions {
        // it wouldn't let me import it so ctrl+c, ctrl+v
        public final double spaceFromCenterToGoalsY = Math.cos(30) * 15.44;
        public static final Pose redUp = new Pose((72 - spaceFromCenterToGoalsX),  botYSize, 90);
        public static final Pose redDown = new Pose((72 - spaceFromCenterToGoalsX), (144 - botYSize), 270);
        public static final Pose blueUp = new Pose((72 + spaceFromCenterToGoalsX), (144 - botYSize), 270);
        public static final Pose blueDown = new Pose((72 + spaceFromCenterToGoalsX), botYSize, 90);
    }

    public static class ParkingZones {
        private static final double offset = 3;
        public static final Pose redUpPark = new Pose((11 - offset), (144 - (GeneralConstants.tileSize * 2) + GeneralConstants.tileSeam + offset));
        public static final Pose redDownPark = new Pose((11 - offset), (144 - GeneralConstants.tileSize - GeneralConstants.tileSeam - offset));
        public static final Pose blueUpPark = new Pose((144 - 11 + offset), ((GeneralConstants.tileSize * 2) - GeneralConstants.tileSeam - offset));
        public static final Pose blueDownPark = new Pose((144 - 11 + offset), (GeneralConstants.tileSize + GeneralConstants.tileSeam + offset));
    }
}
