package frc.team166.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.CommandRobot;
import frc.team166.chopshoplib.DashboardUtils;
import frc.team166.chopshoplib.RobotUtils;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.controls.ButtonJoystick;
import frc.team166.chopshoplib.controls.ButtonXboxController;
import frc.team166.chopshoplib.controls.ButtonXboxController.XBoxButton;
import frc.team166.robot.maps.Maverick;
import frc.team166.robot.subsystems.Drive;
import frc.team166.robot.subsystems.LED;
import frc.team166.robot.subsystems.Lift;
import frc.team166.robot.subsystems.Manipulator;

public class Robot extends CommandRobot {
    // Initialize the mapping for the production robot
    public final RobotMap robotMap = new Maverick();

    // Initialize subsystems and their members
    public final LED led = new LED(robotMap.getLEDMap());
    public final Drive drive = new Drive(robotMap);
    public final Manipulator manipulator = new Manipulator(robotMap);
    public final Lift lift = new Lift(robotMap.getLift());

    // Joysticks
    public static final ButtonJoystick leftDriveStick = new ButtonJoystick(0);
    public static final ButtonJoystick rightDriveStick = new ButtonJoystick(1);
    public static final ButtonXboxController xBoxTempest = new ButtonXboxController(2);

    private Command autoCommand;
    final private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotUtils.logTelemetry();
        RobotUtils.clearPreferences();

        autoChooser.addDefault("Default Auto", drive.driveTime(3, 0.6));
        autoChooser.addObject("Mid Auto", midAuto());
        autoChooser.addObject("Cross Line And Drop Cube", crossLineAndDropCube());
        SmartDashboard.putData("Auto mode", autoChooser);
        CameraServer.getInstance()
                .startAutomaticCapture();

        DashboardUtils.initialize(this);

        xBoxTempest.getButton(XBoxButton.Y)
                .whenPressed(manipulator.CloseOuterManipulator());
        xBoxTempest.getButton(XBoxButton.X)
                .whenPressed(manipulator.OpenOuterManipulator());

        xBoxTempest.getButton(XBoxButton.A)
                .whileHeld(manipulator.ManipulatorIntakeHeld());
        xBoxTempest.getButton(XBoxButton.B)
                .whileHeld(manipulator.ManipulatorDischargeHeld());

        rightDriveStick.getButton(1)
                .whenPressed(manipulator.OpenInnerManipulator());
        rightDriveStick.getButton(2)
                .whenPressed(manipulator.CloseInnerManipulator());
    }

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    @Override
    public void disabledInit() {
        drive.reset();
        lift.reset();
        manipulator.reset();

    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString code to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        autoCommand = autoChooser.getSelected();

        // schedule the autonomous command (example)
        if (autoCommand != null) {
            autoCommand.start();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //
        autoCommand.cancel();
    }

    public Command crossLineAndDropCube() {
        return new CommandChain("Cross Line And Drop Cube").then(lift.moveLiftByInches(-1))
                .then(drive.driveTime(3.6, 0.6), lift.moveLiftByInches(26))
                .then(manipulator.CubeEject());
    }

    public Command midAuto() {
        final String gameData = DriverStation.getInstance()
                .getGameSpecificMessage();
        double degrees = 0.0;
        if (gameData.length() > 0) {
            if (gameData.charAt(0) == 'R') {
                // "R" is for RIGHT NOT RED
                // turning right
                degrees = 90;

            } else {
                // turning left
                degrees = -90.00;
            }
        }
        return new CommandChain("Mid Auto").then(drive.driveTime(.75, .6))
                .then(drive.turnByDegrees(degrees))
                .then(drive.driveTime(.5, .6))
                .then(drive.turnByDegrees(-degrees))
                .then(drive.driveTime(.3, .6), lift.raiseLiftALittle());
    }

    public Command rumble(final XboxController controller) {
        return new TimedCommand("Rumble", 0.1) {
            @Override
            protected void initialize() {
                controller.setRumble(RumbleType.kLeftRumble, 1);
                controller.setRumble(RumbleType.kRightRumble, 1);
            }

            @Override
            protected void end() {
                controller.setRumble(RumbleType.kLeftRumble, 0);
                controller.setRumble(RumbleType.kRightRumble, 0);
            }
        };
    }

    public Command cubePickupWithLights(final int blinkCount) {
        return new CommandChain("Cube Pickup with Lights").then(manipulator.CubePickup())
                .then(rumble(xBoxTempest))
                .then(led.blinkGreen(blinkCount));
    }

}
