package frc.team166.robot.subsystems

import com.chopshop166.chopshoplib.Display
import com.chopshop166.chopshoplib.Resettable
import com.chopshop166.chopshoplib.commands.SetCommand
import com.chopshop166.chopshoplib.outputs.SendableSpeedController

import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.Hand
import edu.wpi.first.wpilibj.SpeedControllerGroup
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.PIDSubsystem
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team166.robot.Robot
import frc.team166.robot.RobotMap

public final class Manipulator(val map : RobotMap) :
    PIDSubsystem("Manipulator", 0.0, 0.0, 0.0, 0.0),
    Resettable
{
    private val deploymentMotor = map.getDeploymentMotor()
    private val rollers = map.getRollers()
    private val innerSolenoid = map.getInnerManipSolenoid()
    private val outerSolenoid = map.getOuterManipSolenoid()
    private val irSensor = map.getManipIrSensor()
    private val potentiometer = map.getManipPotentiometer()

    enum class State {
        CLOSED,
        OPEN
    }

    enum class MotorState(val value : Double) {
        INTAKE(-0.6),
        STOPPED(0.0),
        DISCHARGE(0.6)
    }

    init {
        setAbsoluteTolerance(5.0)
    }

    // METHODS
    override public fun reset() {
        rollers.stopMotor()
        deploymentMotor.stopMotor()
    }

    private var innerState = State.CLOSED
        get() = field
        set(value) {
            if (value == State.OPEN) {
                innerSolenoid.set(Value.kForward)
            } else {
                innerSolenoid.set(Value.kReverse)
            }
            field = value
        }

    private var outerState = State.CLOSED
        get() = field
        set(value) {
            if (value == State.OPEN) {
                outerSolenoid.set(Value.kForward)
            } else {
                outerSolenoid.set(Value.kReverse)
            }
            field = value
        }

    private var motorSpeed = MotorState.STOPPED
        get() = field
        set(value) {
            rollers.set(value.value)
            field = value
        }

    private fun getIRDistance() = irSensor.getVoltage()

    override protected fun returnPIDInput() = potentiometer.pidGet()

    override protected fun usePIDOutput(output : Double) {
        deploymentMotor.set(output)
    }

    // COMMANDS
    override fun initDefaultCommand() {
        defaultCommand = DeployManipulatorWithJoystick(Robot.COPILOT)
    }

    @Display
    fun OpenOuterManipulator() = SetCommand("Open Outer", this, State.OPEN, ::outerState::set)

    @Display
    fun CloseOuterManipulator() = SetCommand("Close Outer", this, State.CLOSED, ::outerState::set)

    @Display
    fun OpenInnerManipulator() = SetCommand("Open Inner", this, State.OPEN, ::innerState::set)

    @Display
    fun CloseInnerManipulator() = SetCommand("Close Inner", this, State.CLOSED, ::innerState::set)

    @Display
    fun ManipulatorDischarge() = SetCommand("DisCharge", this, MotorState.DISCHARGE, ::motorSpeed::set)

    @Display
    fun ManipulatorIntake() = SetCommand("Intake", this, MotorState.INTAKE, ::motorSpeed::set)

    fun ManipulatorIntakeHeld() =
        object : Command("Intake", this) {

            override protected fun initialize() {
                motorSpeed = MotorState.INTAKE
            }

            override protected fun isFinished() = false

            override protected fun end() {
                motorSpeed = MotorState.STOPPED
            }
        }

    fun ManipulatorDischargeHeld() =
        object : Command("Discharge", this) {

            override protected fun initialize() {
                motorSpeed = MotorState.DISCHARGE
            }

            override protected fun isFinished() = false

            override protected fun end() {
                motorSpeed = MotorState.STOPPED
            }
        }

    fun CubeEject() =
        object : TimedCommand("Eject Cube", 5.0, this) {

            override protected fun initialize() {
                var gameData = DriverStation.getInstance().getGameSpecificMessage()

                if (gameData.length > 0 && gameData.get(0) == 'R') {
                    motorSpeed = MotorState.DISCHARGE
                }
            }

            override protected fun end() {
                motorSpeed = MotorState.STOPPED
            }
        }

    fun CubePickup() =
        object : Command("Pick Up Cube", this) {

            override protected fun initialize() {
                outerState = State.OPEN
                motorSpeed = MotorState.INTAKE
            }

            override protected fun isFinished() = getIRDistance() > 0.5

            override protected fun end() {
                outerState = State.CLOSED
                motorSpeed = MotorState.STOPPED
            }
        }

    fun DeployManipulator() =
        object : Command("Deploy Manipulator", this) {

            override protected fun initialize() {
                setpoint = 2.5
            }

            override protected fun isFinished() = onTarget()
        }

    fun DeployManipulatorWithJoystick(controller : XboxController) =
        object : Command("Deploy Manipulator With Joystick", this) {
            override protected fun initialize() {
                disable()
            }

            override protected fun execute() {
                val yval = controller.getY(Hand.kLeft)
                // Sign Preserving Square
                deploymentMotor.set(Math.copySign(yval * yval, yval))
            }

            override protected fun isFinished() = false
        }

    fun enablePID() = InstantCommand("Enable PID", this, ::enable)
}