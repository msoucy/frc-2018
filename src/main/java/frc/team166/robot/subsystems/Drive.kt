package frc.team166.robot.subsystems

import com.chopshop166.chopshoplib.Display
import com.chopshop166.chopshoplib.Resettable
import com.chopshop166.chopshoplib.commands.CommandChain
import com.chopshop166.chopshoplib.sensors.Lidar

import edu.wpi.first.wpilibj.AnalogGyro
import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.command.TimedCommand
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team166.robot.Robot
import frc.team166.robot.RobotMap

public final class Drive(map : RobotMap) : Subsystem(), Resettable {

    private val frontLidar = map.getDriveLidar()
    private val tempestGyro = map.getDriveGyro()
    private val driveTrain = DifferentialDrive(map.getLeftWheelMotors(), map.getRightWheelMotors())

    // defines values that will be used in the PIDController (In order of where they
    // will fall in the Controller)
    private val P = 0.015
    private val I = 0.00005
    private val D = 0.0
    private val F = 0.0
    private val ABS_TOLERANCE_ANGLE = 3.0

    // defines a new double that is going to be used in the line that defines the
    // drive type
    private var angleCorrection = 0.0

    // PIDController loop used to find the power of the motors needed to keep the
    // angle of the gyro at 0
    private val pidController = PIDController(P, I, D, F, tempestGyro) { value : Double ->
        angleCorrection = value
    }

    init {
        pidController.setOutputRange(-0.6, 0.6)
        pidController.setPercentTolerance(0.90)
        pidController.disable()
        pidController.setInputRange(0.0, 360.0)
        pidController.setContinuous()
        pidController.setAbsoluteTolerance(ABS_TOLERANCE_ANGLE)
    }

    // the default command for this code is supposed to rotate the robot so that
    // it's gyro value is 0
    override fun initDefaultCommand() {
        defaultCommand = joystickArcadeTwoStick(Robot.LEFT_DRIVE_STICK, Robot.RIGHT_DRIVE_STICK)
    }

    override fun reset() {
        driveTrain.stopMotor()
    }

    fun xboxArcade(controller : XboxController) =
        object : Command("XBoxArcade", this) {
            override protected fun execute() {
                driveTrain.arcadeDrive(-controller.getY(Hand.kLeft), controller.getX(Hand.kRight))
            }

            override protected fun isFinished() = false
        }

    fun joystickArcadeTwoStick(left : Joystick, right : Joystick) =
        object : Command("Joystick Arcade with two sticks", this) {
            override protected fun execute() {
                driveTrain.arcadeDrive(-left.getY() * 0.8, right.getX())
            }

            override protected fun isFinished() = false
        }

    fun driveStraight(controller : XboxController) =
        object : Command("Drive Straight", this) {
            override protected fun initialize() {
                pidController.reset()
                pidController.setpoint = tempestGyro.getAngle()
                pidController.enable()
            }

            override protected fun execute() {
                driveTrain.arcadeDrive(controller.getTriggerAxis(Hand.kRight)
                        - controller.getTriggerAxis(Hand.kLeft), angleCorrection)
            }

            override protected fun isFinished() = false

            override protected fun end() {
                pidController.disable()
            }
        }

    fun drivetoProximity(inches : Double) =
        object : Command("Drive Distance", this) {
            override protected fun initialize() {
                pidController.setSetpoint(tempestGyro.getAngle())
                pidController.reset()
                pidController.enable()
            }

            override protected fun execute() {
                driveTrain.arcadeDrive(ABS_TOLERANCE_ANGLE, angleCorrection)
            }

            override protected fun isFinished() = frontLidar.getDistance(Lidar.MeasurementType.INCHES) <= inches

            override protected fun end() {
                pidController.disable()
            }
        }

    @Display(value = [45.0], name = "Turn Right 45")
    // @Display(value = [-45.0], name = "Turn Left 45")
    // @Display(value = [90.0], name = "Turn Right 90")
    // @Display(value = [-90.0], name = "Turn Left 90")
    fun turnByDegrees(degrees : Double) =
        object : Command("Turn " + degrees, this) {
            override protected fun initialize() {
                tempestGyro.reset()
                pidController.reset()
                pidController.setAbsoluteTolerance(ABS_TOLERANCE_ANGLE)
                pidController.setpoint = degrees
                pidController.enable()
            }

            override protected fun execute() {
                SmartDashboard.putNumber("Drive Angle", angleCorrection)
                driveTrain.arcadeDrive(0.0, angleCorrection)
            }

            override protected fun isFinished() = pidController.onTarget()

            override protected fun end() {
                pidController.disable()
            }
        }

    @Display(2.0, 0.6)
    fun driveTime(seconds : Double, speed : Double) =
        object : TimedCommand("Drive " + seconds + "s", seconds, this) {
            override protected fun initialize() {
                pidController.reset()
                // drivePidController.setSetpoint(tempestGyro.getAngle())
                // drivePidController.enable()
            }

            override protected fun execute() {
                driveTrain.arcadeDrive(speed, angleCorrection)
            }

            override protected fun end() {
                pidController.disable()
                driveTrain.stopMotor()
            }
        }

    fun driveBox() : Command {
        val box = CommandChain("Box Drive")
        box.then(driveTime(1.0, 0.8))
                .then(turnByDegrees(90.0))
                .then(driveTime(0.5, 0.8))
                .then(turnByDegrees(90.0))
                .then(driveTime(1.0, 0.8))
                .then(turnByDegrees(90.0))
                .then(driveTime(0.5, 0.8))
                .then(turnByDegrees(90.0))
        return box
    }
}
