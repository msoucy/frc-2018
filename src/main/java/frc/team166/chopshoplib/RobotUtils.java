package frc.team166.chopshoplib;

import java.util.stream.DoubleStream;

public final class RobotUtils {
    private RobotUtils() {
    }

    public static Double[] toBoxed(final double... args) {
        return DoubleStream.of(args)
                .boxed()
                .toArray(Double[]::new);
    }
}