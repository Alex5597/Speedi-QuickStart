package org.firstinspires.ftc.teamcode.core.Util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

public class VelocityAdapter {
    private long lastUpdateTime;
    private Pose lastPose;

    public VelocityAdapter() {
        this(new Pose());
    }

    public VelocityAdapter(Pose initialPose) {
        lastUpdateTime = System.nanoTime();
        lastPose = initialPose;
    }


    public Vector getVelocity(Pose pose) {
        long now = System.nanoTime();
        double deltaTimeSeconds = (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;
        if (deltaTimeSeconds <= 0) return new Vector(0, 0, 0);
        Pose twist = pose.subtract(lastPose);
        lastPose = pose;
        return new Vector(twist.getX(DistanceUnit.CM) / deltaTimeSeconds, twist.getY(DistanceUnit.CM) / deltaTimeSeconds, twist.getHeading(AngleUnit.RADIANS) / deltaTimeSeconds);
    }
}
