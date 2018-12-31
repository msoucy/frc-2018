package frc.team166.robot.maps

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX

import edu.wpi.first.wpilibj.AnalogGyro
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.I2C.Port
import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj.SpeedControllerGroup
import com.chopshop166.chopshoplib.outputs.DigitalOutputDutyCycle
import com.chopshop166.chopshoplib.outputs.SendableSpeedController
import com.chopshop166.chopshoplib.sensors.Lidar
import frc.team166.robot.RobotMap

public class Maverick : RobotMap {

    // Core
    private val compressor = Compressor(1)
    // LED
    private val led = MaverickLED()
    // Drive
    private val leftGroup = SpeedControllerGroup(WPI_TalonSRX(8), WPI_TalonSRX(9))
    private val rightGroup = SpeedControllerGroup(WPI_TalonSRX(4), WPI_TalonSRX(5))
    private val driveLidar = Lidar(Port.kOnboard, 0x10)
    private val driveGyro = AnalogGyro(1)
    // Manipulator
    private val leftRoller = WPI_VictorSPX(3)
    private val rightRoller = WPI_TalonSRX(2)
    private val rollers = SpeedControllerGroup(leftRoller, rightRoller)
    private val deploymentMotor = WPI_VictorSPX(1)
    private val innerManipSolenoid = DoubleSolenoid(3, 2)
    private val outerManipSolenoid = DoubleSolenoid(1, 0)
    private val manipIrSensor = AnalogInput(2)
    private val manipPotentiometer = AnalogPotentiometer(3)
    // Lift
    private val lift = MaverickLift()

    init {
        leftRoller.setInverted(false)
        rightRoller.setInverted(true)
    }

    override fun getCompressor() = compressor

    override fun getLEDMap() : RobotMap.LEDMap = led

    public class MaverickLED : RobotMap.LEDMap {
        private val redLED = DigitalOutputDutyCycle(4)
        private val greenLED = DigitalOutputDutyCycle(5)
        private val blueLED = DigitalOutputDutyCycle(6)

        override fun getRed() = redLED

        override fun getGreen() = greenLED

        override fun getBlue() = blueLED
    }

    override fun getLeftWheelMotors() = leftGroup

    override fun getRightWheelMotors() = rightGroup

    override fun getDriveLidar() = driveLidar

    override fun getDriveGyro() : AnalogGyro {
        driveGyro.setSensitivity(0.0125 / 5.45)
        return driveGyro
    }

    override fun getRollers() = rollers

    override fun getInnerManipSolenoid() = innerManipSolenoid

    override fun getOuterManipSolenoid() = outerManipSolenoid

    override fun getDeploymentMotor() : SendableSpeedController {
        deploymentMotor.setInverted(true)
        return SendableSpeedController.wrap(deploymentMotor)
    }

    override fun getManipIrSensor() = manipIrSensor

    override fun getManipPotentiometer() = manipPotentiometer

    public class MaverickLift : RobotMap.LiftMap {
        private val bottomLimitSwitch = DigitalInput(9)
        private val topLimitSwitch = DigitalInput(8)

        private val liftEncoder = Encoder(0, 1)

        private val liftMotorA = WPI_VictorSPX(6)
        private val liftMotorB = WPI_VictorSPX(7)
        private val liftDrive = SpeedControllerGroup(liftMotorA, liftMotorB)

        private val liftBrake = DoubleSolenoid(7, 6)
        private val liftTransmission = DoubleSolenoid(4, 5)

        private val lidar = Lidar(Port.kOnboard, 0x60)

        override fun getMotors() = SendableSpeedController.wrap(liftDrive)

        override fun getTopLimit() = topLimitSwitch

        override fun getBottomLimit() = bottomLimitSwitch

        override fun getEncoder() : Encoder {
            // This is for one inch
            liftEncoder.setDistancePerPulse(0.01636)
            return liftEncoder
        }

        override fun getBrake() = liftBrake

        override fun getShifter() = liftTransmission

        override fun getLidar() = lidar
    }

    override fun getLift() = lift

}
