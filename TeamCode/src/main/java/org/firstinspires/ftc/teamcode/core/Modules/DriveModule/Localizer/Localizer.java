package org.firstinspires.ftc.teamcode.core.Modules.DriveModule.Localizer;

import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

public interface Localizer {

    void update();

    Vector getVelocity();

    Pose getPoseEstimate();

    Pose getPredictedPoseEstimate();

    void resetPosition(Pose startPose);

    void updateOnlyImu();
    Vector getVelocityVector();
    Vector getRawVelocity();
}
