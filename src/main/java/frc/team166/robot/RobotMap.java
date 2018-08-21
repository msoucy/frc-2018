package frc.team166.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.chopshoplib.outputs.SendableSpeedController;
import frc.team166.chopshoplib.sensors.Lidar;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Sendable;

import java.lang.reflect.Method;
import java.lang.reflect.InvocationTargetException;

public interface RobotMap {

    Compressor getCompressor();

    LEDMap getLEDMap();

    interface LEDMap {
        DigitalOutputDutyCycle getRed();

        DigitalOutputDutyCycle getGreen();

        DigitalOutputDutyCycle getBlue();
    }

    // #region drive
    SpeedController getLeftWheelMotors();

    SpeedController getRightWheelMotors();

    Lidar getDriveLidar();

    AnalogGyro getDriveGyro();
    // #endregion

    // #region Manipulator
    SpeedControllerGroup getRollers();

    DoubleSolenoid getInnerManipSolenoid();

    DoubleSolenoid getOuterManipSolenoid();

    SendableSpeedController getDeploymentMotor();

    AnalogInput getManipIrSensor();

    AnalogPotentiometer getManipPotentiometer();
    // #endregion

    LiftMap getLift();

    interface LiftMap extends AutoChildren {
        SendableSpeedController getMotors();

        DigitalInput getTopLimit();

        DigitalInput getBottomLimit();

        Encoder getEncoder();

        DoubleSolenoid getBrake();

        DoubleSolenoid getShifter();

        Lidar getLidar();
    }

    public static interface AutoChildren {
        default public void addChildren(Subsystem system) {
            Class<? extends AutoChildren> aClass = this.getClass();
            for (Method elem : aClass.getMethods()) {
                try {
                    // See if the returned object implements sendable.
                    // If it does then lets add it as a child.
                    if (Sendable.class.isAssignableFrom(elem.getReturnType())) {
                        system.addChild((Sendable) elem.invoke(this));
                    }
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                } catch (InvocationTargetException e) {
                    e.printStackTrace();
                }

            }
        }
    }
}