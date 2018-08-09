package frc.team166.chopshoplib.outputs;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public interface SendableSpeedController extends Sendable, SpeedController {
    static <T extends Sendable & SpeedController> SendableSpeedController wrap(T t) {
        return new SendableSpeedController() {

            @Override
            public String getName() {
                return t.getName();
            }

            @Override
            public void setName(String name) {
                t.setName(name);
            }

            @Override
            public String getSubsystem() {
                return t.getSubsystem();
            }

            @Override
            public void setSubsystem(String subsystem) {
                t.setSubsystem(subsystem);
            }

            @Override
            public void initSendable(SendableBuilder builder) {
                t.initSendable(builder);
            }

            @Override
            public void set(double speed) {
                t.set(speed);
            }

            @Override
            public double get() {
                return t.get();
            }

            @Override
            public void setInverted(boolean isInverted) {
                t.setInverted(isInverted);
            }

            @Override
            public boolean getInverted() {
                return t.getInverted();
            }

            @Override
            public void disable() {
                t.disable();
            }

            @Override
            public void stopMotor() {
                t.stopMotor();
            }

            @Override
            public void pidWrite(double output) {
                t.pidWrite(output);
            }
        };
    }
}
