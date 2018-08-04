package frc.team166.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.chopshoplib.sensors.Lidar;

public interface RobotMap {
    public static class CAN {
        public final static int LIFT_MOTOR_A = 6;
        public final static int LIFT_MOTOR_B = 7;
    }

    public static class Solenoids {
        public final static int LIFT_TRANSMISSION_A = 4;
        public final static int LIFT_TRANSMISSION_B = 5;
        public final static int LIFT_BRAKE_A = 7;
        public final static int LIFT_BRAKE_B = 6;
    }

    public static class DigitalInputs {
        public final static int LIFT_LIMIT_SWITCH_BOTTOM = 9;
        public final static int LIFT_LIMIT_SWITCH_TOP = 8;
        public final static int LIFT_A = 0;
        public final static int LIFT_B = 1;
    }

    public Compressor getCompressor();

    // #region LED
    public DigitalOutputDutyCycle getRedLED();

    public DigitalOutputDutyCycle getGreenLED();

    public DigitalOutputDutyCycle getBlueLED();
    // #endregion

    // #region drive
    public SpeedController getLeftWheelMotors();

    public SpeedController getRightWheelMotors();

    public Lidar getDriveLidar();

    public AnalogGyro getDriveGyro();
    // #endregion

    // #region Manipulator
    public SpeedControllerGroup getRollers();

    public DoubleSolenoid getInnerManipSolenoid();

    public DoubleSolenoid getOuterManipSolenoid();

    public SpeedController getDeploymentMotor();

    public AnalogInput getManipIrSensor();

    public AnalogPotentiometer getManipPotentiometer();
    // #endregion

    // #region
    // #endregion
}