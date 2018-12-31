package frc.team166.robot.subsystems

import java.util.Random

import com.chopshop166.chopshoplib.Display
import com.chopshop166.chopshoplib.outputs.DigitalOutputDutyCycle

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team166.robot.RobotMap

public final class LED(val map : RobotMap.LEDMap) : Subsystem() {

    private val redPort = map.getRed()
    private var red : Boolean
        get() = redPort.get()
        set(value) = redPort.set(value)
    private val greenPort = map.getGreen()
    private var green : Boolean
        get() = greenPort.get()
        set(value) = greenPort.set(value)
    private val bluePort = map.getBlue()
    private var blue : Boolean
        get() = bluePort.get()
        set(value) = bluePort.set(value)

    override public fun initDefaultCommand() {
        defaultCommand = breathTeamColor()
    }

    init {
        SmartDashboard.putData("All Off", InstantCommand("OFF GERALD", this, ::allOff))
    }

    // METHODS
    private fun allOff() {
        red = false
        green = false
        blue = false
    }

    private val isBlueTeam : Boolean
        get() {
            val team = DriverStation.getInstance().getAlliance()
            return team == Alliance.Blue
        }
    
    private val thisTeamColor : DigitalOutputDutyCycle
        get() = if (isBlueTeam) bluePort else redPort
    private val otherTeamColor : DigitalOutputDutyCycle
        get() = if (isBlueTeam) redPort else bluePort

    private var teamColor : Boolean
        get() = thisTeamColor.get()
        set(value) {
            thisTeamColor.set(value)
            otherTeamColor.set(false)
        }

    // COMMANDS
    fun blinkGreen(numberOfBlinks : Int) =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var isOn = true
            private var count = 0

            override protected fun initialize() {
                green = true
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 250) {
                    lastUpdateTime = System.currentTimeMillis()
                    if (isOn) {
                        green = false
                        count++
                    } else {
                        green = true
                    }
                    isOn = !isOn
                }
            }

            override protected fun isFinished() = count >= numberOfBlinks

            override protected fun end() {
                green = false
            }
        }

    fun blinkTeamColor() =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var isOn = true

            override protected fun initialize() {
                teamColor = true
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 750) {
                    lastUpdateTime = System.currentTimeMillis()
                    isOn = !isOn
                    teamColor = isOn
                }
            }

            override protected fun isFinished() = false

            override protected fun end() {
                teamColor = false
            }
        }

    private fun colorOn(color : DigitalOutputDutyCycle) =
        object : Command(this) {
            override protected fun initialize() {
                color.set(true)
            }

            override protected fun isFinished() = false

            override protected fun end() {
                color.set(false)
            }
        }

    fun redOn() = colorOn(redPort)

    fun greenOn() = colorOn(greenPort)

    fun blueOn() = colorOn(bluePort)

    fun cyanOn() =
        object : Command(this) {
            override protected fun initialize() {
                red = false
                blue = true
                green = true
            }

            override protected fun isFinished() = false

            override protected fun end() {
                allOff()
            }
        }

    fun lightTeamColor() =
        object : Command(this) {
            override protected fun initialize() {
                teamColor = true
            }

            override protected fun isFinished() = false

            override protected fun end() {
                teamColor = false
            }
        }

    fun breath(color : DigitalOutputDutyCycle, frequency : Int) =
        object : Command("fade", this) {
            // Approx how often execute is called
            private val EXEC_PERIOD = 20 * 0.001
            private val DUTY_CYCLE_CHANGE = 2.0
            private val period = 1.0 / frequency
            private val changeAmount = DUTY_CYCLE_CHANGE / (period / EXEC_PERIOD)
            private var isIncreasing = true

            override protected fun initialize() {
                color.disablePWM()
            }

            override protected fun execute() {
                color.updateDutyCycle(
                    if (isIncreasing) { color.getPWMRate() + changeAmount }
                    else { color.getPWMRate() - changeAmount }
                )
                if (color.getPWMRate() >= 1 || color.getPWMRate() <= 0) {
                    isIncreasing = !isIncreasing
                }

            }

            override protected fun isFinished() = false
        }

    @Display
    fun breathTeamColor() =
        InstantCommand("Breath Team Color", this) {
            greenPort.disablePWM()
            if (isBlueTeam) {
                redPort.disablePWM()
                breath(bluePort, 2).start()
            } else {
                bluePort.disablePWM()
                breath(redPort, 2).start()
            }
        }

    @Display(1000.0)
    fun notSeizure(numberOfBlinks : Double) =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var isOn = true
            private var count = 0

            override protected fun initialize() {
                blue = true
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 15) {
                    lastUpdateTime = System.currentTimeMillis()
                    if (isOn) {
                        blue = false
                        red = true
                        count++
                    } else {
                        blue = true
                        red = false
                    }
                    isOn = !isOn
                }
            }

            override protected fun isFinished() = count >= numberOfBlinks

            override protected fun end() {
                red = false
                blue = false
            }
        }

    @Display(1000.0)
    public fun rainbow(numberOfBlinks : Double) =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var count = 0
            private val rand = Random()

            override protected fun initialize() {
                blue = true
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 15) {
                    lastUpdateTime = System.currentTimeMillis()
                    blue = rand.nextBoolean()
                    red = rand.nextBoolean()
                    green = rand.nextBoolean()
                    count++
                }
            }

            override protected fun isFinished() = count >= numberOfBlinks

            override protected fun end() {
                allOff()
            }
        }
}
