package org.firstinspires.ftc.teamcode.core.Util.utils;

public abstract class NanoClock {

    /**
     * Returns the number of seconds since an arbitrary (yet consistent) origin.
     */
    public abstract double seconds();

    /**
     * Returns a NanoClock backed by System.nanoTime.
     */
    public static NanoClock system() {
        return new NanoClock() {
            @Override
            public double seconds() {
                return System.nanoTime() / 1e9;
            }
        };
    }
}