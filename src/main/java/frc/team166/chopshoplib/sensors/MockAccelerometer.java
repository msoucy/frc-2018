package frc.team166.chopshoplib.sensors;

import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class MockAccelerometer extends SensorBase implements Accelerometer {

    private double x;
    private double y;
    private double z;
    private Range range;

    public void setX(final double newX) {
        x = newX;
    }

    public void setY(final double newY) {
        y = newY;
    }

    public void setZ(final double newZ) {
        z = newZ;
    }

    public Range getRange() {
        return range;
    }

    @Override
    public void setRange(final Range range) {
        this.range = range;
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getZ() {
        return z;
    }

    @Override
    public void initSendable(final SendableBuilder builder) {
        builder.setSmartDashboardType("Accelerometer");
        builder.addDoubleProperty("X", this::getX, this::setX);
        builder.addDoubleProperty("Y", this::getY, this::setY);
        builder.addDoubleProperty("Z", this::getZ, this::setZ);
    }
}