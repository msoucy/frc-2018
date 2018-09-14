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
import frc.team166.chopshoplib.Display;
import frc.team166.chopshoplib.Resettable;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.sensors.Lidar;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;

public final class Drive extends Subsystem implements Resettable {

    private final Lidar frontLidar;
    private final AnalogGyro tempestGyro;
    private final DifferentialDrive driveTrain;

    // defines values that will be used in the PIDController (In order of where they
    // will fall in the Controller)
    private final static double P = 0.015;
    private final static double I = 0.00005;
    private final static double D = 0;
    private final static double F = 0;
    private final static double ABS_TOLERANCE_ANGLE = 3;

    // defines a new double that is going to be used in the line that defines the
    // drive type
    private double angleCorrection;

    // PIDController loop used to find the power of the motors needed to keep the
    // angle of the gyro at 0
    private final PIDController pidController;

    // this makes children that control the tempestGyro, drive motors, and
    // PIDController loop.
    public Drive(final RobotMap map) {
        super();

        driveTrain = new DifferentialDrive(map.getLeftWheelMotors(), map.getRightWheelMotors());
        tempestGyro = map.getDriveGyro();
        frontLidar = map.getDriveLidar();

        pidController = new PIDController(P, I, D, F, tempestGyro, (double value) -> {
            // this assigns the output to the angle (double) defined later in the code)
            angleCorrection = value;
        });

        tempestGyro.setSensitivity(0.0125 / 5.45);
        pidController.setOutputRange(-0.6, 0.6);
        pidController.setPercentTolerance(0.90);

        pidController.disable();
        pidController.setInputRange(0, 360);
        pidController.setContinuous();
        pidController.setAbsoluteTolerance(ABS_TOLERANCE_ANGLE);
    }

    // the default command for this code is supposed to rotate the robot so that
    // it's gyro value is 0
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(joystickArcadeTwoStick(Robot.LEFT_DRIVE_STICK, Robot.RIGHT_DRIVE_STICK));
    }

    @Override
    public void reset() {
        driveTrain.stopMotor();
    }

    public Command xboxArcade(final XboxController controller) {
        return new Command("XBoxArcade", this) {
            @Override
            protected void execute() {
                driveTrain.arcadeDrive(-controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command joystickArcadeTwoStick(final Joystick left, final Joystick right) {
        return new Command("Joystick Arcade with two sticks", this) {
            @Override
            protected void execute() {
                driveTrain.arcadeDrive(-left.getY() * 0.8, right.getX());
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

        };
    };

    public Command driveStraight(final XboxController controller) {
        return new Command("Drive Straight", this) {
            @Override
            protected void initialize() {
                pidController.reset();
                pidController.setSetpoint(tempestGyro.getAngle());
                pidController.enable();
            }

            @Override
            protected void execute() {
                driveTrain.arcadeDrive(controller.getTriggerAxis(Hand.kRight)
                        - controller.getTriggerAxis(Hand.kLeft), angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return false;

            }

            @Override
            protected void end() {
                pidController.disable();
            }
        };
    }

    public Command drivetoProximity(final double inches) {
        return new Command("Drive Distance", this) {
            @Override
            protected void initialize() {
                pidController.setSetpoint(tempestGyro.getAngle());
                pidController.reset();
                pidController.enable();
            }

            @Override
            protected void execute() {
                driveTrain.arcadeDrive(ABS_TOLERANCE_ANGLE, angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return frontLidar.getDistance(Lidar.MeasurementType.INCHES) <= inches;

            }

            @Override
            protected void end() {
                pidController.disable();
            }
        };
    }

    @Display(value = 45.0, name = "Turn Right 45")
    @Display(value = -45.0, name = "Turn Left 45")
    @Display(value = 90.0, name = "Turn Right 90")
    @Display(value = -90.0, name = "Turn Left 90")
    public Command turnByDegrees(final double degrees) {
        return new Command("Turn " + degrees, this) {
            @Override
            protected void initialize() {
                tempestGyro.reset();
                pidController.reset();
                pidController.setAbsoluteTolerance(ABS_TOLERANCE_ANGLE);
                pidController.setSetpoint(degrees);
                pidController.enable();
            }

            @Override
            protected void execute() {
                SmartDashboard.putNumber("Drive Angle", angleCorrection);
                driveTrain.arcadeDrive(0.0, angleCorrection);

            }

            @Override
            protected boolean isFinished() {
                return pidController.onTarget();
            }

            @Override
            protected void end() {
                pidController.disable();
            }
        };
    }

    @Display({ 2.0, 0.6 })
    public Command driveTime(final double seconds, final double speed) {
        return new Command("Drive " + seconds + "s", this) {
            @Override
            protected void initialize() {
                pidController.reset();
                // drivePidController.setSetpoint(tempestGyro.getAngle());
                // drivePidController.enable();
                setTimeout(seconds);
            }

            @Override
            protected void execute() {
                driveTrain.arcadeDrive(speed, angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }

            @Override
            protected void end() {
                pidController.disable();
                driveTrain.stopMotor();
            }
        };
    }

    public Command driveBox() {
        final CommandChain box = new CommandChain("Box Drive");
        box.then(driveTime(1, .8))
                .then(turnByDegrees(90))
                .then(driveTime(.5, .8))
                .then(turnByDegrees(90))
                .then(driveTime(1, .8))
                .then(turnByDegrees(90))
                .then(driveTime(.5, .8))
                .then(turnByDegrees(90));
        return box;
    }
}
