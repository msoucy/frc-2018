package frc.team166.robot.subsystems;

import com.chopshop166.chopshoplib.Display;
import com.chopshop166.chopshoplib.Resettable;
import com.chopshop166.chopshoplib.commands.CommandChain;
import com.chopshop166.chopshoplib.outputs.TankDriveSubsystem;
import com.chopshop166.chopshoplib.sensors.Lidar;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public final class Drive extends TankDriveSubsystem implements Resettable {

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
    private final static double MAX_VELOCITY = 1.7;

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

    @Override
    public TankModifier getModifier(final Trajectory trajectory) {
        return new TankModifier(trajectory).modify(0.8);
    }

    @Override
    public DifferentialDrive getDriveTrain() {
        return driveTrain;
    }

    @Override
    public Gyro getGyro() {
        return tempestGyro;
    }

    @Override
    public void configureLeftEncoderFollower(final EncoderFollower follower) {
        follower.configureEncoder(0, 1024, 0.2);
        follower.configurePIDVA(1.0, 0.0, 0.0, 1.0 / MAX_VELOCITY, 0);
    }

    @Override
    public Integer getLeftEncoder() {
        return 0;
    }

    @Override
    public Integer getRightEncoder() {
        return 0;
    }

    public Command driveSample() {
        final Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH, 0.05, MAX_VELOCITY, 2.0, 60.0);
        return this.path(config)
                .then(-4, -1, Pathfinder.d2r(-45))
                .then(-2, -2, 0)
                .then(0, 0, 0)
                .compile("Drive Sample", this);
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
        return new TimedCommand("Drive " + seconds + "s", seconds, this) {
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
