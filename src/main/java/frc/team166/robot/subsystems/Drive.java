package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;

public class Drive extends Subsystem {

    Lidar frontLidar;
    AnalogGyro tempestGyro;
    DifferentialDrive m_drive;

    // defines values that will be used in the PIDController (In order of where they
    // will fall in the Controller)
    final static double kP = 0.015;
    final static double kI = 0.00005;
    final static double kD = 0;
    final static double kF = 0;
    final static double AUTOMATIC_ROBOT_FORWARD_SPEED = .2;
    final static double ABSOLUTE_TOLERANCE_ANGLE = 3;

    // defines a new double that is going to be used in the line that defines the
    // drive type
    double angleCorrection;

    // PIDController loop used to find the power of the motors needed to keep the
    // angle of the gyro at 0
    PIDController drivePidController;

    // this makes children that control the tempestGyro, drive motors, and
    // PIDController loop.
    public Drive(RobotMap map) {

        m_drive = new DifferentialDrive(map.getLeftWheelMotors(), map.getRightWheelMotors());
        tempestGyro = map.getDriveGyro();
        frontLidar = map.getDriveLidar();

        drivePidController = new PIDController(kP, kI, kD, kF, tempestGyro,
            (double value) -> {
                // this assigns the output to the angle (double) defined later in the code)
                angleCorrection = value;
            });

        // SmartDashboard.putData("XBox", XboxArcade());
        // SmartDashboard.putData("Turn -45", TurnByDegrees(-45));
        // SmartDashboard.putData("Turn 45", TurnByDegrees(45));
        // SmartDashboard.putData("Drive 2s", DriveTime(2, .6));
        // SmartDashboard.putData("Drive Box", DriveBox());

        addChild(tempestGyro);
        addChild(m_drive);
        addChild(drivePidController);
        addChild("Front LiDAR", frontLidar);
        tempestGyro.setSensitivity(0.0125 / 5.45);
        drivePidController.setOutputRange(-0.6, 0.6);
        drivePidController.setPercentTolerance(0.90);

        drivePidController.disable();
        drivePidController.setInputRange(0, 360);
        drivePidController.setContinuous();
        drivePidController.setAbsoluteTolerance(ABSOLUTE_TOLERANCE_ANGLE);
    }

    // the default command for this code is supposed to rotate the robot so that
    // it's gyro value is 0
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(JoystickArcadeTwoStick(Robot.leftDriveStick, Robot.rightDriveStick));
    }

    public void reset() {
        m_drive.stopMotor();
    }

    public Command XboxArcade(final XboxController controller) {
        return new SubsystemCommand("XBoxArcade", this) {
            @Override
            protected void execute() {
                m_drive.arcadeDrive(-controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command JoystickArcadeTwoStick(final Joystick left, final Joystick right) {
        return new SubsystemCommand("joystick Arcade with two sticks", this) {
            @Override
            protected void execute() {
                m_drive.arcadeDrive(-left.getY() * 0.8, right.getX());
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

        };
    };

    public Command DriveStraight(final XboxController controller) {
        return new SubsystemCommand("Drive Straight", this) {
            @Override
            protected void initialize() {
                drivePidController.reset();
                drivePidController.setSetpoint(tempestGyro.getAngle());
                drivePidController.enable();
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(controller.getTriggerAxis(Hand.kRight)
                        - controller.getTriggerAxis(Hand.kLeft), angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return false;

            }

            @Override
            protected void end() {
                drivePidController.disable();
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    public Command DrivetoProximity(double inches) {
        return new SubsystemCommand("Drive Distance", this) {
            @Override
            protected void initialize() {
                drivePidController.setSetpoint(tempestGyro.getAngle());
                drivePidController.reset();
                drivePidController.enable();
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(ABSOLUTE_TOLERANCE_ANGLE, angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return (frontLidar.getDistance(true) <= inches);

            }

            @Override
            protected void end() {
                drivePidController.disable();
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    public Command TurnByDegrees(double degrees) {
        return new SubsystemCommand("Turn " + degrees, this) {
            @Override
            protected void initialize() {
                tempestGyro.reset();
                drivePidController.reset();
                drivePidController.setAbsoluteTolerance(ABSOLUTE_TOLERANCE_ANGLE);
                drivePidController.setSetpoint(degrees);
                drivePidController.enable();
            }

            @Override
            protected void execute() {
                SmartDashboard.putNumber("Drive Angle", angleCorrection);
                m_drive.arcadeDrive(0.0, angleCorrection);

            }

            @Override
            protected boolean isFinished() {
                return drivePidController.onTarget();
            }

            @Override
            protected void end() {
                drivePidController.disable();
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    public Command DriveTime(double seconds, double speed) {
        return new SubsystemCommand("Drive Time", this) {
            @Override
            protected void initialize() {
                drivePidController.reset();
                // drivePidController.setSetpoint(tempestGyro.getAngle());
                // drivePidController.enable();
                setTimeout(seconds);
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(speed, angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }

            @Override
            protected void end() {
                drivePidController.disable();
                m_drive.stopMotor();
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    public Command DriveBox() {
        return new CommandChain("Box Drive").then(DriveTime(1, .8))
                .then(TurnByDegrees(90))
                .then(DriveTime(.5, .8))
                .then(TurnByDegrees(90))
                .then(DriveTime(1, .8))
                .then(TurnByDegrees(90))
                .then(DriveTime(.5, .8))
                .then(TurnByDegrees(90));

    }

}
