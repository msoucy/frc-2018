package frc.team166.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.controls.ButtonJoystick;
import frc.team166.chopshoplib.controls.ButtonXboxController;
import frc.team166.chopshoplib.controls.ButtonXboxController.XBoxButton;
import frc.team166.robot.maps.Maverick;
import frc.team166.robot.subsystems.Drive;
import frc.team166.robot.subsystems.LED;
import frc.team166.robot.subsystems.Lift;
import frc.team166.robot.subsystems.Manipulator;

public class Robot extends TimedRobot {
    // Initialize the mapping for the production robot
    public static final RobotMap robotMap = new Maverick();

    // Initialize subsystems and their members
    public static final LED led = new LED(robotMap);
    public static final Drive drive = new Drive(robotMap);
    public static final Manipulator manipulator = new Manipulator(robotMap);
    public static final Lift lift = new Lift(robotMap.getLift());

    // Joysticks
    public static final ButtonJoystick leftDriveStick = new ButtonJoystick(0);
    public static final ButtonJoystick rightDriveStick = new ButtonJoystick(1);
    public static final ButtonXboxController xBoxTempest = new ButtonXboxController(2);

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        m_chooser.addDefault("Default Auto", drive.DriveTime(3, 0.6));
        m_chooser.addObject("Mid Auto", midAuto());
        m_chooser.addObject("Cross Line And Drop Cube", crossLineAndDropCube());
        SmartDashboard.putData("Auto mode", m_chooser);
        SmartDashboard.putData("Turn 90", drive.TurnByDegrees(90));
        SmartDashboard.putData("Turn -90", drive.TurnByDegrees(-90));
        CameraServer.getInstance()
                .startAutomaticCapture();

        xBoxTempest.getButton(XBoxButton.kY)
                .whenPressed(manipulator.CloseOuterManipulator());
        xBoxTempest.getButton(XBoxButton.kX)
                .whenPressed(manipulator.OpenOuterManipulator());

        xBoxTempest.getButton(XBoxButton.kA)
                .whileHeld(manipulator.ManipulatorIntakeHeld());
        xBoxTempest.getButton(XBoxButton.kB)
                .whileHeld(manipulator.ManipulatorDischargeHeld());

        rightDriveStick.getButton(1)
                .whenPressed(manipulator.CubeDrop());
        rightDriveStick.getButton(2)
                .whenPressed(manipulator.CubeClamp());
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

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance()
                .run();
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
        m_autonomousCommand = m_chooser.getSelected();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.start();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance()
                .run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //
        m_autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance()
                .run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        // Do nothing special.
    }


    void logTelemetry() {
        final String branch = getResource("branch.txt");
        SmartDashboard.putString("branch", branch);

        final String commit = getResource("commit.txt");
        SmartDashboard.putString("commit", commit);

        final String changes = getResource("changes.txt");
        SmartDashboard.putString("changes", changes);

        final String buildtime = getResource("buildtime.txt");
        SmartDashboard.putString("buildtime", buildtime);
    }

    String getResource(final String path) {
        try(InputStream stream = getClass().getResourceAsStream("/" + path);
            InputStreamReader reader = new InputStreamReader(stream, StandardCharsets.UTF_8);
            BufferedReader bufferedReader = new BufferedReader(reader)
        ) {
            return bufferedReader.lines()
                .collect(Collectors.joining("\n"));
        } catch(IOException e) {
            return "";
        }
    }

    public Command crossLineAndDropCube() {
        return new CommandChain("Cross Line And Drop Cube").then(lift.MoveLiftByInches(-1))
                .then(drive.DriveTime(3.6, 0.6), lift.MoveLiftByInches(26))
                .then(manipulator.CubeEject());
    }

    public Command midAuto() {
        final String gameData = DriverStation.getInstance().getGameSpecificMessage();
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
        return new CommandChain("Mid Auto").then(drive.DriveTime(.75, .6))
                .then(drive.TurnByDegrees(degrees))
                .then(drive.DriveTime(.5, .6))
                .then(drive.TurnByDegrees(-degrees))
                .then(drive.DriveTime(.3, .6), lift.RaiseLiftALittle());
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
                .then(led.BlinkGreen(blinkCount));
    }

}
