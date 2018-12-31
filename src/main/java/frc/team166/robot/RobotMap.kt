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

    fun getCompressor() : Compressor

    fun getLEDMap() : LEDMap

    interface LEDMap {
        fun getRed() : DigitalOutputDutyCycle

        fun getGreen() : DigitalOutputDutyCycle

        fun getBlue() : DigitalOutputDutyCycle
    }

    // #region drive
    fun getLeftWheelMotors() : SpeedController

    fun getRightWheelMotors() : SpeedController

    fun getDriveLidar() : Lidar

    fun getDriveGyro() : AnalogGyro
    // #endregion

    // #region Manipulator
    fun getRollers() : SpeedControllerGroup

    fun getInnerManipSolenoid() : DoubleSolenoid

    fun getOuterManipSolenoid() : DoubleSolenoid

    fun getDeploymentMotor() : SendableSpeedController

    fun getManipIrSensor() : AnalogInput

    fun getManipPotentiometer() : AnalogPotentiometer
    // #endregion

    fun getLift() : LiftMap

    interface LiftMap {
        fun getMotors() : SendableSpeedController

        fun getTopLimit() : DigitalInput

        fun getBottomLimit() : DigitalInput

        fun getEncoder() : Encoder

        fun  getBrake() : DoubleSolenoid

        fun getShifter() : DoubleSolenoid

        fun getLidar() : Lidar
    }
}