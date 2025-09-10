package org.firstinspires.ftc.teamcode.core.Util.utils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer.TwoWheelLocalizer;

public class DashboardPoseTracker {
    private double[] xPositions;
    private double[] yPositions;
    private Localizer poseUpdater;
    private long lastUpdateTime;
    private final int TRACKING_LENGTH = 1500;
    private final long UPDATE_TIME = 50;
    private final int TRACKING_SIZE = TRACKING_LENGTH / (int) UPDATE_TIME;

    public DashboardPoseTracker(Localizer poseUpdater) {
        this.poseUpdater = poseUpdater;
        xPositions = new double[TRACKING_SIZE];
        yPositions = new double[TRACKING_SIZE];
        for (int i = 0; i < TRACKING_SIZE; i++) {
            xPositions[i] = poseUpdater.getPoseEstimate().getY(DistanceUnit.INCH);
            yPositions[i] = -poseUpdater.getPoseEstimate().getX(DistanceUnit.INCH);
        }
        lastUpdateTime = System.currentTimeMillis() - UPDATE_TIME;
    }

    public void update() {
        if (System.currentTimeMillis() - lastUpdateTime > UPDATE_TIME) {
            lastUpdateTime = System.currentTimeMillis();
            for (int i = TRACKING_SIZE - 1; i > 0; i--) {
                xPositions[i] = xPositions[i - 1];
                yPositions[i] = yPositions[i - 1];
            }
            xPositions[0] = poseUpdater.getPoseEstimate().getY(DistanceUnit.INCH);
            yPositions[0] = -poseUpdater.getPoseEstimate().getX(DistanceUnit.INCH);
        }
    }

    public double[] getXPositionsArray() {
        return xPositions;
    }

    public double[] getYPositionsArray() {
        return yPositions;
    }
}