/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
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
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.robot.subsystems.Drive;
import frc.team166.robot.subsystems.LED;
import frc.team166.robot.subsystems.Lift;
import frc.team166.robot.subsystems.Manipulator;

public class Robot extends TimedRobot {
    // Initialize subsystems and their members
    public static final LED led = new LED(4, 5, 6);
    public static final Drive drive = new Drive();
    public static final Manipulator manipulator = new Manipulator();
    public static final Lift lift = new Lift();
    public static final Compressor compressy = new Compressor(1);

    // Joysticks
    public static ButtonJoystick leftDriveStick = new ButtonJoystick(0);
    public static ButtonJoystick rightDriveStick = new ButtonJoystick(1);
    public static ButtonXboxController xBoxTempest = new ButtonXboxController(2);

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        m_chooser.addDefault("Default Auto", drive.DriveTime(3, 0.6));
        m_chooser.addObject("Mid Auto", MidAuto());
        m_chooser.addObject("Cross Line And Drop Cube", CrossLineAndDropCube());
        SmartDashboard.putData("Auto mode", m_chooser);
        SmartDashboard.putData("Turn 90", drive.TurnByDegrees(90));
        SmartDashboard.putData("Turn -90", drive.TurnByDegrees(-90));
        CameraServer.getInstance().startAutomaticCapture();

        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kY).whenPressed(manipulator.CloseOuterManipulator());
        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kX).whenPressed(manipulator.OpenOuterManipulator());

        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kA).whileHeld(manipulator.ManipulatorIntakeHeld());
        xBoxTempest.getButton(ButtonXboxController.xBoxButton.kB).whileHeld(manipulator.ManipulatorDischargeHeld());

        rightDriveStick.getButton(1).whenPressed(manipulator.CubeDrop());
        rightDriveStick.getButton(2).whenPressed(manipulator.CubeClamp());
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
        Scheduler.getInstance().run();
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
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public Command CrossLineAndDropCube() {
        return new CommandChain("Cross Line And Drop Cube").then(lift.MoveLiftByInches(-1))
                .then(drive.DriveTime(3.6, 0.6), lift.MoveLiftByInches(26)).then(manipulator.CubeEject());
    }

    public Command MidAuto() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        double degrees = 0.0;
        if (gameData.length() > 0) {
            if (gameData.charAt(0) == 'R') {
                // "R" is for RIGHT NOT RED
                degrees = 90;
                // turning right
                System.out.println("Right");

            } else {
                degrees = -90.00;
                // turning left
                System.out.println("Left");
            }
        }
        Command cmdMidAuto = new CommandChain("Mid Auto").then(drive.DriveTime(.75, .6))
                .then(drive.TurnByDegrees(degrees)).then(drive.DriveTime(.5, .6)).then(drive.TurnByDegrees(-degrees))
                .then(drive.DriveTime(.3, .6), lift.RaiseLiftALittle());
        return cmdMidAuto;

    }

    public Command Rumble(final XboxController controller) {

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

    public Command CubePickupWithLights(int blinkCount) {
        return new CommandChain("Cube Pickup with Lights").then(manipulator.CubePickup()).then(Rumble(xBoxTempest))
                .then(led.BlinkGreen(blinkCount));
    }

}
