package org.firstinspires.ftc.teamcode.core.Util.Hardware;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.core.Util.utils.NanoClock;

public class VelocityAdapterForPinPointAndRev {
    private final static int CPS_STEP = 0x10000;
    private final NanoClock clock;
    private double lastPosition;
    private double velocityEstimates = 0, lastVelocity = 0;
    private double lastUpdateTime;

    public VelocityAdapterForPinPointAndRev() {
        clock = NanoClock.system();

        this.lastPosition = 0;
        this.lastUpdateTime = clock.seconds();
    }
    public double getVelocityBasedOnTicks(@NonNull double valueInTicks) {
        if (valueInTicks != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            if (dt == 0) return lastVelocity;
            velocityEstimates = (valueInTicks - lastPosition) / dt;
            lastVelocity = velocityEstimates;
            lastPosition = valueInTicks;
            lastUpdateTime = currentTime;
        }
        return velocityEstimates;
    }
}
