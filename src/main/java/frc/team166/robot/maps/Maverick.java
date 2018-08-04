package frc.team166.robot.maps;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
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
    SpeedControllerGroup leftGroup = new SpeedControllerGroup(new WPI_VictorSPX(8), new WPI_VictorSPX(9));
    SpeedControllerGroup rightGroup = new SpeedControllerGroup(new WPI_VictorSPX(4), new WPI_VictorSPX(5));
    Lidar driveLidar = new Lidar(Port.kOnboard, 0x10);
    AnalogGyro driveGyro = new AnalogGyro(1);
    // Manipulator
    WPI_VictorSPX leftRoller = new WPI_VictorSPX(3);
    WPI_TalonSRX rightRoller = new WPI_TalonSRX(2);
    SpeedControllerGroup rollers = new SpeedControllerGroup(leftRoller, rightRoller);

    DoubleSolenoid innerManipSolenoid = new DoubleSolenoid(3, 2);
    DoubleSolenoid outerManipSolenoid = new DoubleSolenoid(1, 0);

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

}