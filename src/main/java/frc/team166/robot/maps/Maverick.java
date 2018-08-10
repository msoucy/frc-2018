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
    private final Compressor compressor = new Compressor(1);
    // LED
    private final DigitalOutputDutyCycle redLED = new DigitalOutputDutyCycle(4);
    private final DigitalOutputDutyCycle greenLED = new DigitalOutputDutyCycle(5);
    private final DigitalOutputDutyCycle blueLED = new DigitalOutputDutyCycle(6);
    // Drive
    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(new WPI_TalonSRX(8),
            new WPI_TalonSRX(9));
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(new WPI_TalonSRX(4),
            new WPI_TalonSRX(5));
    private final Lidar driveLidar = new Lidar(Port.kOnboard, 0x10);
    private final AnalogGyro driveGyro = new AnalogGyro(1);
    // Manipulator
    private final WPI_VictorSPX leftRoller = new WPI_VictorSPX(3);
    private final WPI_TalonSRX rightRoller = new WPI_TalonSRX(2);
    private final SpeedControllerGroup rollers = new SpeedControllerGroup(leftRoller, rightRoller);
    private final WPI_VictorSPX deploymentMotor = new WPI_VictorSPX(1);
    private final DoubleSolenoid innerManipSolenoid = new DoubleSolenoid(3, 2);
    private final DoubleSolenoid outerManipSolenoid = new DoubleSolenoid(1, 0);
    private final AnalogInput manipIrSensor = new AnalogInput(2);
    private final AnalogPotentiometer manipPotentiometer = new AnalogPotentiometer(3);
    // Lift
    private final LiftMap lift = new MaverickLift();

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
        DigitalInput bottomLimitSwitch = new DigitalInput(9);
        DigitalInput topLimitSwitch = new DigitalInput(8);

        Encoder liftEncoder = new Encoder(0, 1);

        WPI_VictorSPX liftMotorA = new WPI_VictorSPX(6);
        WPI_VictorSPX liftMotorB = new WPI_VictorSPX(7);
        SpeedControllerGroup liftDrive = new SpeedControllerGroup(liftMotorA, liftMotorB);

        DoubleSolenoid liftBrake = new DoubleSolenoid(7, 6);
        DoubleSolenoid liftTransmission = new DoubleSolenoid(4, 5);

        Lidar lidar = new Lidar(Port.kOnboard, 0x60);

        @Override
        public SendableSpeedController getMotors() {
            return SendableSpeedController.wrap(liftDrive);
        }

        @Override
        public DigitalInput getTopLimit() {
            return topLimitSwitch;
        }

        @Override
        public DigitalInput getBottomLimit() {
            return bottomLimitSwitch;
        }

        @Override
        public Encoder getEncoder() {
            return liftEncoder;
        }

        @Override
        public DoubleSolenoid getBrake() {
            return liftBrake;
        }

        @Override
        public DoubleSolenoid getShifter() {
            return liftTransmission;
        }

        @Override
        public Lidar getLidar() {
            return lidar;
        }
    }

    @Override
    public LiftMap getLift() {
        return lift;
    }

}