package frc.team166.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.chopshoplib.sensors.Lidar;

public interface RobotMap {
    public static class CAN {
        // changes motor ports into integers
        public final static int ROLLER_LEFT = 3;
        public final static int ROLLER_RIGHT = 2;
        public final static int LIFT_MOTOR_A = 6;
        public final static int LIFT_MOTOR_B = 7;
        public final static int DEPLOYMENT_MOTOR = 1;
    }

    public static class AnalogInputs {
        // changes input ports into integers
        public final static int IR = 2;
        public final static int MANIPULATOR_POTENTIOMETER = 3;
    }

    public static class Solenoids {
        // changes Solenoid ports into integers

        public final static int LIFT_TRANSMISSION_A = 4;
        public final static int LIFT_TRANSMISSION_B = 5;
        public final static int MANIPULATOR_SOLENOID_INNER_A = 3;
        public final static int MANIPULATOR_SOLENOID_INNER_B = 2;
        public final static int MANIPULATOR_SOLENOID_OUTER_A = 1;
        public final static int MANIPULATOR_SOLENOID_OUTER_B = 0;
        public final static int LIFT_BRAKE_A = 7;
        public final static int LIFT_BRAKE_B = 6;
    }

    public static class DigitalInputs {
        // changes digital imput ports into integers
        public final static int LIFT_LIMIT_SWITCH_BOTTOM = 9;
        public final static int LIFT_LIMIT_SWITCH_TOP = 8;
        public final static int LIFT_A = 0;
        public final static int LIFT_B = 1;
    }

    public Compressor getCompressor();

    public DigitalOutputDutyCycle getRedLED();

    public DigitalOutputDutyCycle getGreenLED();

    public DigitalOutputDutyCycle getBlueLED();

    public SpeedController getLeftWheelMotors();

    public SpeedController getRightWheelMotors();

    public Lidar getDriveLidar();

    public AnalogGyro getDriveGyro();

    public LiftMap getLift();

    public interface LiftMap {
        public SpeedController getLiftMotors();

        public DigitalInput getLiftTopLimit();
    
        public DigitalInput getLiftBottomLimit();
        
        public Encoder getLiftEncoder();

        public DoubleSolenoid getLiftBrake();
    
        public DoubleSolenoid getLiftShifter();
    }
}