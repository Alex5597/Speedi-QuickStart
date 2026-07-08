package org.firstinspires.ftc.teamcode.core.Util.Algorithm;

public class PathMotionProfile {
    private final double length;
    private final double acceleration;
    private final double deceleration;
    private final double maxUsedVelocity;
    private final double tAcceleration;
    private final double tCruise;
    private final double tDeceleration;
    private final double accelerationDistance;
    private final double cruiseDistance;

    public PathMotionProfile(double length, double maxVelocity, double acceleration, double deceleration) {
        this.length = Math.max(0, length);
        this.acceleration = Math.max(1e-6, Math.abs(acceleration));
        this.deceleration = Math.max(1e-6, Math.abs(deceleration));
        double velocityLimit = Math.max(1e-6, Math.abs(maxVelocity));

        if (this.length <= 1e-6) {
            maxUsedVelocity = 0;
            tAcceleration = 0;
            tDeceleration = 0;
            tCruise = 0;
            accelerationDistance = 0;
            cruiseDistance = 0;
        } else {
            maxUsedVelocity = Math.min(velocityLimit, Math.sqrt((2.0 * this.length * this.acceleration * this.deceleration) / (this.acceleration + this.deceleration)));
            tAcceleration = maxUsedVelocity / this.acceleration;
            tDeceleration = maxUsedVelocity / this.deceleration;
            tCruise = Math.max(this.length / maxUsedVelocity - (tAcceleration + tDeceleration) / 2.0, 0);
            accelerationDistance = this.acceleration * tAcceleration * tAcceleration / 2.0;
            cruiseDistance = maxUsedVelocity * tCruise;
        }
    }

    public State get(double timeSeconds) {
        double time = Math.max(0, timeSeconds);
        if (length <= 1e-6) return new State(0, 0, 0, true);

        if (time <= tAcceleration) {
            return new State(acceleration * time * time / 2.0, acceleration * time, acceleration, false);
        }

        if (time <= tAcceleration + tCruise) {
            double cruiseTime = time - tAcceleration;
            return new State(accelerationDistance + maxUsedVelocity * cruiseTime, maxUsedVelocity, 0, false);
        }

        if (time <= getDuration()) {
            double decelTime = time - tAcceleration - tCruise;
            double position = accelerationDistance + cruiseDistance + maxUsedVelocity * decelTime - deceleration * decelTime * decelTime / 2.0;
            double velocity = Math.max(0, maxUsedVelocity - deceleration * decelTime);
            return new State(Math.min(length, position), velocity, -deceleration, false);
        }

        return new State(length, 0, 0, true);
    }

    public double getDuration() {
        return tAcceleration + tCruise + tDeceleration;
    }

    public static class State {
        public final double position;
        public final double velocity;
        public final double acceleration;
        public final boolean done;

        public State(double position, double velocity, double acceleration, boolean done) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.done = done;
        }
    }
}
