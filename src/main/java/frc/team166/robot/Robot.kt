package frc.team166.robot

import com.chopshop166.chopshoplib.CommandRobot
import com.chopshop166.chopshoplib.DashboardUtils
import com.chopshop166.chopshoplib.RobotUtils
import com.chopshop166.chopshoplib.commands.CommandChain
import com.chopshop166.chopshoplib.controls.ButtonJoystick
import com.chopshop166.chopshoplib.controls.ButtonXboxController
import com.chopshop166.chopshoplib.controls.ButtonXboxController.XBoxButton

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.TimedCommand
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team166.robot.maps.Maverick
import frc.team166.robot.subsystems.Drive
import frc.team166.robot.subsystems.LED
import frc.team166.robot.subsystems.Lift
import frc.team166.robot.subsystems.Manipulator

public class Robot : CommandRobot() {
    // Initialize the mapping for the production robot
    val robotMap = Maverick()

    // Initialize subsystems and their members
    val led = LED(robotMap.getLEDMap())
    val drive = Drive(robotMap)
    val manipulator = Manipulator(robotMap)
    val lift = Lift(robotMap.getLift())

    // Joysticks
    companion object {
        val LEFT_DRIVE_STICK = ButtonJoystick(0)
        val RIGHT_DRIVE_STICK = ButtonJoystick(1)
        val COPILOT = ButtonXboxController(2)
    }

    private var autoCommand : Command? = null
    private val autoChooser = SendableChooser<Command>()

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    override fun robotInit() {
        DashboardUtils.logTelemetry()
        RobotUtils.clearPreferences()

        autoChooser.setDefaultOption("Default Auto", drive.driveTime(3.0, 0.6))
        autoChooser.addOption("Mid Auto", midAuto())
        autoChooser.addOption("Cross Line And Drop Cube", crossLineAndDropCube())
        SmartDashboard.putData("Auto mode", autoChooser)
        CameraServer.getInstance()
                .startAutomaticCapture()

        DashboardUtils.initialize(this)

        COPILOT.getButton(XBoxButton.Y)
                .whenPressed(manipulator.CloseOuterManipulator())
        COPILOT.getButton(XBoxButton.X)
                .whenPressed(manipulator.OpenOuterManipulator())

        COPILOT.getButton(XBoxButton.A)
                .whileHeld(manipulator.ManipulatorIntakeHeld())
        COPILOT.getButton(XBoxButton.B)
                .whileHeld(manipulator.ManipulatorDischargeHeld())

        RIGHT_DRIVE_STICK.getButton(1)
                .whenPressed(manipulator.OpenInnerManipulator())
        RIGHT_DRIVE_STICK.getButton(2)
                .whenPressed(manipulator.CloseInnerManipulator())
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard.
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings and commands.
     */
    override fun autonomousInit() {
        autoCommand = autoChooser.getSelected()

        autoCommand?.start()
    }

    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //
        autoCommand?.cancel()
    }

    fun crossLineAndDropCube() : Command {
        val chain = CommandChain("Cross Line And Drop Cube")
        chain.then(lift.moveLiftByInches(-1.0))
                .then(drive.driveTime(3.6, 0.6), lift.moveLiftByInches(26.0))
                .then(manipulator.CubeEject())
        return chain
    }

    fun midAuto() : Command {
        val gameData = DriverStation.getInstance().getGameSpecificMessage()
        var degrees = 0.0
        if (gameData.length > 0) {
            if (gameData.get(0) == 'R') {
                // "R" is for RIGHT NOT RED
                // turning right
                degrees = 90.0

            } else {
                // turning left
                degrees = -90.0
            }
        }
        val auto = CommandChain("Mid Auto")
        auto.then(drive.driveTime(0.75, 0.6))
                .then(drive.turnByDegrees(degrees))
                .then(drive.driveTime(0.5, 0.6))
                .then(drive.turnByDegrees(-degrees))
                .then(drive.driveTime(0.3, 0.6), lift.raiseLiftALittle())
        return auto
    }

    fun rumble(controller : XboxController) =
        object : TimedCommand("Rumble", 0.1) {
            override protected fun initialize() {
                controller.setRumble(RumbleType.kLeftRumble, 1.0)
                controller.setRumble(RumbleType.kRightRumble, 1.0)
            }

            override protected fun end() {
                controller.setRumble(RumbleType.kLeftRumble, 0.0)
                controller.setRumble(RumbleType.kRightRumble, 0.0)
            }
        }

    fun cubePickupWithLights(blinkCount : Int) : Command {
        val chain = CommandChain("Cube Pickup with Lights")
        chain.then(manipulator.CubePickup())
                .then(rumble(COPILOT))
                .then(led.blinkGreen(blinkCount))
        return chain
    }

}
