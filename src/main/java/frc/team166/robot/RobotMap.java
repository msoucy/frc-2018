package frc.team166.robot;

import com.chopshop166.chopshoplib.outputs.DigitalOutputDutyCycle;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.Lidar;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

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

    interface LiftMap {
        SendableSpeedController getMotors();

        DigitalInput getTopLimit();

        DigitalInput getBottomLimit();

        Encoder getEncoder();

        DoubleSolenoid getBrake();

        DoubleSolenoid getShifter();

        Lidar getLidar();
    }
}