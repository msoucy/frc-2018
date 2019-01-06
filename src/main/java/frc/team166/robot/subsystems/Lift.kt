package frc.team166.robot.subsystems

import com.chopshop166.chopshoplib.Display
import com.chopshop166.chopshoplib.Resettable
import com.chopshop166.chopshoplib.commands.CommandChain
import com.chopshop166.chopshoplib.commands.SetCommand
import com.chopshop166.chopshoplib.outputs.SendableSpeedController
import com.chopshop166.chopshoplib.sensors.Lidar

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.PIDSubsystem
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team166.robot.Robot
import frc.team166.robot.RobotMap

public final class Lift(val map : RobotMap.LiftMap) :
    PIDSubsystem("Lift", 0.0, 0.0, 0.0, 0.0),
    Resettable
{

    private val bottomLimitSwitch = map.getBottomLimit()
    private val topLimitSwitch = map.getTopLimit()
    private val encoder = map.getEncoder()
    private val liftDrive = map.getMotors()
    private val liftBrake = map.getBrake()
    private val liftTransmission = map.getShifter()
    private val liftLidar = map.getLidar()

    // enumerator that will be pulled from for the GoToHeight Command
    public enum class LiftHeights(val value : Double) {
        // will be changed
        FLOOR(0.0),
        SWITCH(1.0),
        PORTAL(2.0),
        INTAKE(3.0),
        SCALE_LOW(4.0),
        SCALE_HIGH(5.0),
        CLIMB(6.0),
        MAX_HEIGHT(7.0)
    }

    private enum class Gear {
        LOW,
        HIGH
    }

    private enum class BrakeState {
        DISENGAGED,
        ENGAGED
    }

    // sets the maximum lidar distance before switching to the encoder
    private val MAX_LIDAR_DISTANCE = 60.0

    init {
        setOutputRange(-1.0, 1.0)
        setAbsoluteTolerance(0.05)
        addChild(liftHeight)

        liftDrive.inverted = true

        val prefs = Preferences.getInstance()

        if (!prefs.containsKey("Use LIDAR")) {
            prefs.putBoolean("Use LIDAR", false)
        }
    }

    override protected fun returnPIDInput() = liftHeight

    override protected fun usePIDOutput(output : Double) {
        if (!topLimitSwitch.get() && output > 0.0) {
            setSetpoint(LiftHeights.MAX_HEIGHT.value)
            liftDrive.stopMotor()
        } else if (!bottomLimitSwitch.get() && output < 0.0) {
            // liftEncoder.reset()
            setSetpoint(LiftHeights.FLOOR.value)
            liftDrive.stopMotor()
        } else {
            liftDrive.set(output)
        }
    }

    override public fun reset() {
        liftDrive.stopMotor()
    }

    public val liftHeight : Double
        get() {
            var distance = encoder.getDistance()
            val prefs = Preferences.getInstance()
            if (prefs.getBoolean("Use LIDAR", false)) {
                val lidarDistance = liftLidar.getDistance(Lidar.MeasurementType.INCHES)
                if (lidarDistance > MAX_LIDAR_DISTANCE) {
                    distance = lidarDistance
                }
            }
            return distance
        }

    private var gear = Gear.HIGH
        get() = field
        set(value) {
            if (value == Gear.LOW) {
                liftTransmission.set(Value.kForward)
            } else {
                liftTransmission.set(Value.kReverse)
            }
            field = value
        }


    private var brakeState = BrakeState.ENGAGED
        get() = field
        set(value) {
            if (value == BrakeState.ENGAGED) {
                liftBrake.set(Value.kReverse)
            } else {
                liftBrake.set(Value.kForward)
            }
        }

    override fun initDefaultCommand() {
        defaultCommand = manualLift(Robot.COPILOT)
    }

    fun raiseLiftALittle() =
        object : TimedCommand("Raise Lift A Little", 2.5, this) {
            override protected fun initialize() {
                brakeState = BrakeState.DISENGAGED
                liftDrive.set(0.9)
            }

            override protected fun execute() {
                liftDrive.set(0.5)
            }

            override protected fun end() {
                liftDrive.stopMotor()
                brakeState = BrakeState.ENGAGED
            }
        }

    private fun goToHeight(height : LiftHeights, gearState : Gear) =
        InstantCommand(this) {
            brakeState = BrakeState.DISENGAGED
            gear = gearState
            setSetpoint(height.value)
        }

    fun manualLift(controller : XboxController) =
        object : Command(this) {
            override protected fun initialize() {
                disable()
            }

            override protected fun execute() {
                val elevatorControl = controller.getTriggerAxis(Hand.kRight)
                        - controller.getTriggerAxis(Hand.kLeft)

                brakeState =
                    if (elevatorControl >= .1 || elevatorControl <= -0.1) {
                        BrakeState.DISENGAGED
                    } else {
                        BrakeState.ENGAGED
                    }

                liftDrive.set(
                    if (elevatorControl > 0 && !topLimitSwitch.get()) {
                        controller.getTriggerAxis(Hand.kLeft)
                    } else if (elevatorControl < 0 && !bottomLimitSwitch.get()) {
                        controller.getTriggerAxis(Hand.kRight)
                    } else {
                        elevatorControl
                    }
                )
            }

            override protected fun isFinished() = false

            override protected fun end() {
                enable()
            }
        }

    @Display(8.0)
    fun moveLiftByInches(inches : Double) =
        object : Command(this) {
            private var destinationHeight = encoder.getDistance() + inches

            override protected fun initialize() {
                disable()
                brakeState = BrakeState.DISENGAGED
                liftDrive.set(
                    if (destinationHeight > encoder.getDistance()) {
                        0.75
                    } else {
                        -0.50
                    }
                )
            }

            override protected fun isFinished() : Boolean {
                if (liftDrive.get() > 0) {
                    if (encoder.getDistance() >= destinationHeight) {
                        return true
                    }
                } else {
                    if (encoder.getDistance() <= destinationHeight) {
                        return true
                    }
                }
                return false
            }

            override protected fun end() {
                liftDrive.set(0.0)
                brakeState = BrakeState.ENGAGED
            }
        }

    fun goUp() =
        object : Command(this) {
            override protected fun initialize() {
                brakeState = BrakeState.DISENGAGED
            }

            override protected fun execute() {
                setSetpointRelative(1.0)
            }

            override protected fun isFinished() = false
        }

    fun goDown() =
        object : Command(this) {
            override protected fun initialize() {
                brakeState = BrakeState.DISENGAGED
            }

            override protected fun execute() {
                setSetpointRelative(-1.0)
            }

            override protected fun isFinished() = false
        }

    public fun lowerLiftToLimitSwitch() =
        object : Command(this) {
            override protected fun initialize() {
                brakeState = BrakeState.DISENGAGED
            }

            override protected fun execute() {
                liftDrive.set(-0.5)
            }

            override protected fun isFinished() = !bottomLimitSwitch.get()
        }

    public fun climbUp() : Command {
        val chain = CommandChain("Climb Up")
        chain.then(disengageBrake())
                .then(shiftToHighGear())
                .then(goToHeight(LiftHeights.CLIMB, Gear.HIGH))
                .then(shiftToLowGear())
                .then(goToHeight(LiftHeights.SCALE_LOW, Gear.LOW))
                .then(engageBrake())
        return chain
    }

    public fun shiftToHighGear() : Command = SetCommand("Shift To High Gear", this, Gear.HIGH, ::gear::set)

    public fun shiftToLowGear() : Command = SetCommand("Shift To Low Gear", this, Gear.LOW, ::gear::set)

    public fun engageBrake() : Command = SetCommand("Brake", this, BrakeState.ENGAGED, ::brakeState::set)

    fun disengageBrake() : Command = SetCommand("Don't Brake", this, BrakeState.DISENGAGED, ::brakeState::set)
}
