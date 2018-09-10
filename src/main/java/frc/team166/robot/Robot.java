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
    public static final ButtonJoystick LEFT_DRIVE_STICK = new ButtonJoystick(0);
    public static final ButtonJoystick RIGHT_DRIVE_STICK = new ButtonJoystick(1);
    public static final ButtonXboxController COPILOT = new ButtonXboxController(2);

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

        autoChooser.setDefaultOption("Default Auto", drive.driveTime(3, 0.6));
        autoChooser.addOption("Mid Auto", midAuto());
        autoChooser.addOption("Cross Line And Drop Cube", crossLineAndDropCube());
        SmartDashboard.putData("Auto mode", autoChooser);
        CameraServer.getInstance()
                .startAutomaticCapture();

        DashboardUtils.initialize(this);

        COPILOT.getButton(XBoxButton.Y)
                .whenPressed(manipulator.CloseOuterManipulator());
        COPILOT.getButton(XBoxButton.X)
                .whenPressed(manipulator.OpenOuterManipulator());

        COPILOT.getButton(XBoxButton.A)
                .whileHeld(manipulator.ManipulatorIntakeHeld());
        COPILOT.getButton(XBoxButton.B)
                .whileHeld(manipulator.ManipulatorDischargeHeld());

        RIGHT_DRIVE_STICK.getButton(1)
                .whenPressed(manipulator.OpenInnerManipulator());
        RIGHT_DRIVE_STICK.getButton(2)
                .whenPressed(manipulator.CloseInnerManipulator());
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
        final CommandChain chain = new CommandChain("Cross Line And Drop Cube");
        chain.then(lift.moveLiftByInches(-1))
                .then(drive.driveTime(3.6, 0.6), lift.moveLiftByInches(26))
                .then(manipulator.CubeEject());
        return chain;
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
        final CommandChain auto = new CommandChain("Mid Auto");
        auto.then(drive.driveTime(.75, .6))
                .then(drive.turnByDegrees(degrees))
                .then(drive.driveTime(.5, .6))
                .then(drive.turnByDegrees(-degrees))
                .then(drive.driveTime(.3, .6), lift.raiseLiftALittle());
        return auto;
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
        final CommandChain chain = new CommandChain("Cube Pickup with Lights");
        chain.then(manipulator.CubePickup())
                .then(rumble(COPILOT))
                .then(led.blinkGreen(blinkCount));
        return chain;
    }

}
