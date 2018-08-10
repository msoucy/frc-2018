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

public interface RobotMap {

    Compressor getCompressor();

    // #region LED
    DigitalOutputDutyCycle getRedLED();

    DigitalOutputDutyCycle getGreenLED();

    DigitalOutputDutyCycle getBlueLED();
    // #endregion

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

    // #region Lift
    LiftMap getLift();

    interface LiftMap {
        SendableSpeedController getMotors();

        DigitalInput getTopLimit();

        DigitalInput getBottomLimit();

        Encoder getEncoder();

        DoubleSolenoid getBrake();

        DoubleSolenoid getShifter();

        Lidar getLidar();
    }
    // #endregion
}