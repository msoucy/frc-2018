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

    private val red = map.getRed()
    private val green = map.getGreen()
    private val blue = map.getBlue()

    override public fun initDefaultCommand() {
        setDefaultCommand(breathTeamColor())
    }

    init {
        SmartDashboard.putData("All Off", InstantCommand("OFF GERALD", this, ::allOff))
    }

    // METHODS
    private fun allOff() {
        red.set(false)
        green.set(false)
        blue.set(false)
    }

    private fun isBlueTeam() : Boolean {
        val team = DriverStation.getInstance().getAlliance()
        return team == Alliance.Blue
    }

    private fun setTeamColor(turnOn : Boolean) {
        if (isBlueTeam()) {
            red.set(false)
            blue.set(turnOn)
        } else {
            blue.set(false)
            red.set(turnOn)
        }
    }

    // COMMANDS
    fun blinkGreen(numberOfBlinks : Int) =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var isOn = true
            private var count = 0

            override protected fun initialize() {
                green.set(true)
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 250) {
                    lastUpdateTime = System.currentTimeMillis()
                    if (isOn) {
                        green.set(false)
                        count++
                    } else {
                        green.set(true)
                    }
                    isOn = !isOn
                }
            }

            override protected fun isFinished() = count >= numberOfBlinks

            override protected fun end() {
                green.set(false)
            }
        }

    fun blinkTeamColor() =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var isOn = true

            override protected fun initialize() {
                setTeamColor(true)
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 750) {
                    lastUpdateTime = System.currentTimeMillis()
                    isOn = !isOn
                    setTeamColor(isOn)
                }
            }

            override protected fun isFinished() = false

            override protected fun end() {
                setTeamColor(false)
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

    fun redOn() = colorOn(red)

    fun greenOn() = colorOn(green)

    fun blueOn() = colorOn(blue)

    fun cyanOn() =
        object : Command(this) {
            override protected fun initialize() {
                red.set(false)
                blue.set(true)
                green.set(true)
            }

            override protected fun isFinished() = false

            override protected fun end() {
                allOff()
            }
        }

    fun lightTeamColor() =
        object : Command(this) {
            override protected fun initialize() {
                setTeamColor(true)
            }

            override protected fun isFinished() = false

            override protected fun end() {
                setTeamColor(false)
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
                if (isIncreasing) {
                    color.updateDutyCycle(color.getPWMRate() + changeAmount)
                } else {
                    color.updateDutyCycle(color.getPWMRate() - changeAmount)
                }
                if (color.getPWMRate() >= 1 || color.getPWMRate() <= 0) {
                    isIncreasing = !isIncreasing
                }

            }

            override protected fun isFinished() = false
        }

    @Display
    fun breathTeamColor() =
        InstantCommand("Breath Team Color", this) {
            green.disablePWM()
            if (isBlueTeam()) {
                red.disablePWM()
                breath(blue, 2).start()
            } else {
                blue.disablePWM()
                breath(red, 2).start()
            }
        }

    @Display(1000.0)
    fun notSeizure(numberOfBlinks : Double) =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var isOn = true
            private var count = 0

            override protected fun initialize() {
                blue.set(true)
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 15) {
                    lastUpdateTime = System.currentTimeMillis()
                    if (isOn) {
                        blue.set(false)
                        red.set(true)
                        isOn = false
                        count++
                    } else {
                        blue.set(true)
                        red.set(false)
                    }
                    isOn = !isOn
                }
            }

            override protected fun isFinished() = count >= numberOfBlinks

            override protected fun end() {
                red.set(false)
                blue.set(false)
            }
        }

    @Display(1000.0)
    public fun rainbow(numberOfBlinks : Double) =
        object : Command(this) {
            private var lastUpdateTime = System.currentTimeMillis()
            private var count = 0
            private val rand = Random()

            override protected fun initialize() {
                blue.set(true)
            }

            override protected fun execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 15) {
                    lastUpdateTime = System.currentTimeMillis()
                    blue.set(rand.nextBoolean())
                    red.set(rand.nextBoolean())
                    green.set(rand.nextBoolean())
                    count++
                }
            }

            override protected fun isFinished() = count >= numberOfBlinks

            override protected fun end() {
                allOff()
            }
        }
}
