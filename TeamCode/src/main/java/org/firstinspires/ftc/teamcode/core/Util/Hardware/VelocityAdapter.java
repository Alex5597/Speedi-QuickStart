package org.firstinspires.ftc.teamcode.core.Util.Hardware;

import static org.firstinspires.ftc.teamcode.core.Util.utils.Constants.cmPerTickForward;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Util.Math.Pose;
import org.firstinspires.ftc.teamcode.core.Util.Math.Vector;

public class VelocityAdapter {
    private long lastUpdateTime;
    private Pose lastPose;

    public VelocityAdapter() {
        lastUpdateTime = System.nanoTime();
        lastPose = new Pose();
    }


    public Vector getVelocity(Pose pose) {
        double deltaTimeSeconds = (System.nanoTime() - lastUpdateTime) / 1e9;
        lastUpdateTime = System.nanoTime();
        Pose twist = pose.subtract(lastPose);
        lastPose = pose;
        return new Vector(twist.getX() / deltaTimeSeconds, twist.getY() / deltaTimeSeconds);
    }
}
