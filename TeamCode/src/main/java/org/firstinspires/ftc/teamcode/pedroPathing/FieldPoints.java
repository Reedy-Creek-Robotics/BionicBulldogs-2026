package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.FieldPoints.HivePositions.spaceFromCenterToGoalsX;
import static org.firstinspires.ftc.teamcode.robot.BotDimensions.botYSize;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.BotDimensions;

public class FieldPoints {
    public static final PoseFactory poseFactory = PoseFactory.degrees();
    public static class GeneralConstants {
        public static final double tileSize = 144 / 6;
    }
    public static class HivePositions {
        public static final double spaceFromCenterToGoalsX = 25.5 / 2; // space between hives / 2
        public final double spaceFromCenterToGoalsY = Math.cos(30) * 15.44;
        public final Pose redUp = poseFactory.of((72 - spaceFromCenterToGoalsX), (72 - spaceFromCenterToGoalsY),0);
        public final Pose redDown = poseFactory.of((72 - spaceFromCenterToGoalsX), (72 + spaceFromCenterToGoalsY),0);
        public final Pose blueUp = poseFactory.of((72 + spaceFromCenterToGoalsX), (72 - spaceFromCenterToGoalsY),0);
        public final Pose blueDown = poseFactory.of((72 + spaceFromCenterToGoalsX), (72 + spaceFromCenterToGoalsY),0);
    }

    public static class StartingPositions {
        // it wouldn't let me import it so ctrl+c, ctrl+v
        public final double spaceFromCenterToGoalsY = Math.cos(30) * 15.44;
        public static final Pose redUp = poseFactory.of((72 - spaceFromCenterToGoalsX),  (botYSize/2), 90);
        public static final Pose redDown = poseFactory.of((72 - spaceFromCenterToGoalsX), (144 - (botYSize/2)), 270);
        public static final Pose blueUp = poseFactory.of((72 + spaceFromCenterToGoalsX), (144 - (botYSize/2)), 270);
        public static final Pose blueDown = poseFactory.of((72 + spaceFromCenterToGoalsX), (botYSize/2), 90);
    }

    public static class ParkingZones {
        private static final double offset = 3;
        public static final Pose redUpPark = poseFactory.of((11 - offset + (BotDimensions.botXSize / 2)), (144 - (GeneralConstants.tileSize * 2) + offset - (botYSize / 2)) , 90);//(11 - offset), (144 - (GeneralConstants.tileSize * 2) + GeneralConstants.tileSeam + offset),180);
        public static final Pose redDownPark = poseFactory.of((11 - offset + (BotDimensions.botXSize / 2)), (144 - GeneralConstants.tileSize - offset + (botYSize / 2)),180);
        public static final Pose blueUpPark = poseFactory.of((144 - 11 + offset), ((GeneralConstants.tileSize * 2) - offset),0);
        public static final Pose blueDownPark = poseFactory.of((144 - 11 + offset), (GeneralConstants.tileSize + offset),0);
    }
}
