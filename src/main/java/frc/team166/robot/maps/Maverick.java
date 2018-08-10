package frc.team166.robot.maps;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.chopshoplib.outputs.SendableSpeedController;
import frc.team166.chopshoplib.sensors.Lidar;
import frc.team166.robot.RobotMap;

public class Maverick implements RobotMap {

    // Core
    Compressor compressor = new Compressor(1);
    // LED
    DigitalOutputDutyCycle redLED = new DigitalOutputDutyCycle(4);
    DigitalOutputDutyCycle greenLED = new DigitalOutputDutyCycle(5);
    DigitalOutputDutyCycle blueLED = new DigitalOutputDutyCycle(6);
    // Drive
    SpeedControllerGroup leftGroup = new SpeedControllerGroup(new WPI_TalonSRX(8),
            new WPI_TalonSRX(9));
    SpeedControllerGroup rightGroup = new SpeedControllerGroup(new WPI_TalonSRX(4),
            new WPI_TalonSRX(5));
    Lidar driveLidar = new Lidar(Port.kOnboard, 0x10);
    AnalogGyro driveGyro = new AnalogGyro(1);
    // Manipulator
    WPI_VictorSPX leftRoller = new WPI_VictorSPX(3);
    WPI_TalonSRX rightRoller = new WPI_TalonSRX(2);
    SpeedControllerGroup rollers = new SpeedControllerGroup(leftRoller, rightRoller);
    WPI_VictorSPX deploymentMotor = new WPI_VictorSPX(1);
    DoubleSolenoid innerManipSolenoid = new DoubleSolenoid(3, 2);
    DoubleSolenoid outerManipSolenoid = new DoubleSolenoid(1, 0);
    AnalogInput manipIrSensor = new AnalogInput(2);
    AnalogPotentiometer manipPotentiometer = new AnalogPotentiometer(3);
    // Lift
    LiftMap lift = new MaverickLift();

    public Maverick() {
        leftRoller.setInverted(false);
        rightRoller.setInverted(true);
    }

    @Override
    public Compressor getCompressor() {
        return compressor;
    }

    @Override
    public DigitalOutputDutyCycle getRedLED() {
        return redLED;
    }

    @Override
    public DigitalOutputDutyCycle getGreenLED() {
        return greenLED;
    }

    @Override
    public DigitalOutputDutyCycle getBlueLED() {
        return blueLED;
    }

    @Override
    public SpeedController getLeftWheelMotors() {
        return leftGroup;
    }

    @Override
    public SpeedController getRightWheelMotors() {
        return rightGroup;
    }

    @Override
    public Lidar getDriveLidar() {
        return driveLidar;
    }

    @Override
    public AnalogGyro getDriveGyro() {
        return driveGyro;
    }

    @Override
    public SpeedControllerGroup getRollers() {
        return rollers;
    }

    @Override
    public DoubleSolenoid getInnerManipSolenoid() {
        return innerManipSolenoid;
    }

    @Override
    public DoubleSolenoid getOuterManipSolenoid() {
        return outerManipSolenoid;
    }

    @Override
    public SendableSpeedController getDeploymentMotor() {
        return SendableSpeedController.wrap(deploymentMotor);
    }

    @Override
    public AnalogInput getManipIrSensor() {
        return manipIrSensor;
    }

    @Override
    public AnalogPotentiometer getManipPotentiometer() {
        return manipPotentiometer;
    }

    public static class MaverickLift implements LiftMap {
        DigitalInput bottomLimitSwitch = new DigitalInput(
                RobotMap.DigitalInputs.LIFT_LIMIT_SWITCH_BOTTOM);
        DigitalInput topLimitSwitch = new DigitalInput(
                RobotMap.DigitalInputs.LIFT_LIMIT_SWITCH_TOP);

        Encoder liftEncoder = new Encoder(RobotMap.DigitalInputs.LIFT_A,
                RobotMap.DigitalInputs.LIFT_B);

        WPI_VictorSPX liftMotorA = new WPI_VictorSPX(RobotMap.CAN.LIFT_MOTOR_A);
        WPI_VictorSPX liftMotorB = new WPI_VictorSPX(RobotMap.CAN.LIFT_MOTOR_B);
        SpeedControllerGroup liftDrive = new SpeedControllerGroup(liftMotorA, liftMotorB);

        DoubleSolenoid liftBrake = new DoubleSolenoid(RobotMap.Solenoids.LIFT_BRAKE_A,
                RobotMap.Solenoids.LIFT_BRAKE_B);
        DoubleSolenoid liftTransmission = new DoubleSolenoid(RobotMap.Solenoids.LIFT_TRANSMISSION_A,
                RobotMap.Solenoids.LIFT_TRANSMISSION_B);

        @Override
        public SendableSpeedController getLiftMotors() {
            return SendableSpeedController.wrap(liftDrive);
        }

        @Override
        public DigitalInput getLiftTopLimit() {
            return topLimitSwitch;
        }

        @Override
        public DigitalInput getLiftBottomLimit() {
            return bottomLimitSwitch;
        }

        @Override
        public Encoder getLiftEncoder() {
            return liftEncoder;
        }

        @Override
        public DoubleSolenoid getLiftBrake() {
            return liftBrake;
        }

        @Override
        public DoubleSolenoid getLiftShifter() {
            return liftTransmission;
        }
    }

    @Override
    public LiftMap getLift() {
        return lift;
    }

}