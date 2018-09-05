package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.team166.chopshoplib.DashboardUtils;
import frc.team166.chopshoplib.Display;
import frc.team166.chopshoplib.Resettable;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.outputs.SendableSpeedController;
import frc.team166.chopshoplib.sensors.Lidar;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;

public final class Lift extends PIDSubsystem implements Resettable {
    // This is for one inch
    private static final double encoderDistancePerTick = 0.01636;

    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;
    private final Encoder Encoder;
    private final SendableSpeedController liftDrive;
    private final DoubleSolenoid liftBrake;
    private final DoubleSolenoid liftTransmission;

    // these define the PID values for the lift
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 0;

    // this adds the LIDAR sensor
    private Lidar liftLidar;

    // enumerator that will be pulled from for the GoToHeight Command
    public enum LiftHeights {
        // will be changed
        kFloor(0),
        kSwitch(1),
        kPortal(2),
        kIntake(3),
        kScaleLow(4),
        kScaleHigh(5),
        kClimb(6),
        kMaxHeight(7);

        private double value;

        LiftHeights(final double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }

    // sets the maximum lidar distance before switching to the encoder
    private final static double kMaxLidarDistance = 60;

    public Lift(final RobotMap.LiftMap map) {
        super("Lift", kP, kI, kD, kF);
        bottomLimitSwitch = map.getBottomLimit();
        topLimitSwitch = map.getTopLimit();
        Encoder = map.getEncoder();
        liftDrive = map.getMotors();
        liftBrake = map.getBrake();
        liftTransmission = map.getShifter();
        liftLidar = map.getLidar();

        DashboardUtils.initialize(this);

        setOutputRange(-1, 1);
        setAbsoluteTolerance(0.05);
        Encoder.setDistancePerPulse(encoderDistancePerTick);
        addChild(findLiftHeight());

        liftDrive.setInverted(true);

        final Preferences prefs = Preferences.getInstance();

        if (!prefs.containsKey("Use LIDAR")) {
            prefs.putBoolean("Use LIDAR", false);
        }
    }

    @Override
    protected double returnPIDInput() {
        return findLiftHeight();
    }

    @Override
    protected void usePIDOutput(final double output) {
        if (!topLimitSwitch.get() && output > 0) {
            setSetpoint(LiftHeights.kMaxHeight.get());
            liftDrive.stopMotor();
        } else if (!bottomLimitSwitch.get() && output < 0) {
            // liftEncoder.reset();
            setSetpoint(LiftHeights.kFloor.get());
            liftDrive.stopMotor();
        } else {
            liftDrive.set(output);
        }
    }

    @Override
    public void reset() {
        liftDrive.stopMotor();
    }

    public double findLiftHeight() {
        double distance = Encoder.getDistance();
        if (Preferences.getInstance()
                .getBoolean("Use LIDAR", false)
                && liftLidar.getDistance(true) > kMaxLidarDistance) {
            distance = liftLidar.getDistance(true);
        }
        return distance;
    }

    // does not do anything
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(manualLift(Robot.xBoxTempest));
    }

    public Command raiseLiftALittle() {
        return new SubsystemCommand("Raise Lift A Little", this) {
            @Override
            protected void initialize() {
                setTimeout(2.5);
                doDisengageBrake();
                liftDrive.set(0.9);
            }

            @Override
            protected void execute() {
                liftDrive.set(0.5);
            }

            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }

            @Override
            protected void end() {
                liftDrive.stopMotor();
                doEngageBrake();
                liftDrive.set(0);
            }
        };
    }

    public Command goToHeight(final LiftHeights height, final boolean isHighGear) {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                doDisengageBrake();
                if (isHighGear) {
                    setGear(Gear.High);
                } else {
                    setGear(Gear.Low);
                }
                setSetpoint(height.get());
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
        };
    }

    public Command manualLift(final XboxController controller) {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disable();
            }

            @Override
            protected void execute() {
                final double elevatorControl = controller.getTriggerAxis(Hand.kRight)
                        - controller.getTriggerAxis(Hand.kLeft);

                if (elevatorControl >= .1 || elevatorControl <= -0.1) {
                    doDisengageBrake();
                } else {
                    doEngageBrake();

                }
                if (elevatorControl > 0 && !topLimitSwitch.get()) {
                    liftDrive.set(controller.getTriggerAxis(Hand.kLeft));
                } else if (elevatorControl < 0 && !bottomLimitSwitch.get()) {
                    liftDrive.set(controller.getTriggerAxis(Hand.kRight));
                } else {
                    liftDrive.set(elevatorControl);
                }

            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                enable();
            }
        };
    }

    @Display(8)
    public Command moveLiftByInches(final double inches) {
        return new SubsystemCommand(this) {
            private double destinationHeight;

            @Override
            protected void initialize() {
                disable();
                doDisengageBrake();
                destinationHeight = Encoder.getDistance() + inches;
                if (destinationHeight > Encoder.getDistance()) {
                    liftDrive.set(0.75);
                } else {
                    liftDrive.set(-0.50);
                }
            }

            @Override
            protected boolean isFinished() {
                if (liftDrive.get() > 0) {
                    if (Encoder.getDistance() >= destinationHeight) {
                        return true;
                    }
                } else {
                    if (Encoder.getDistance() <= destinationHeight) {
                        return true;
                    }
                }
                return false;
            }

            @Override
            protected void end() {
                liftDrive.set(0);
                doEngageBrake();
            }
        };
    }

    public Command goUp() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                doDisengageBrake();
            }

            @Override
            protected void execute() {
                setSetpointRelative(1);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command goDown() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                doDisengageBrake();
            }

            @Override
            protected void execute() {
                setSetpointRelative(-1);

            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    public Command lowerLiftToLimitSwitch() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                doDisengageBrake();
            }

            @Override
            protected void execute() {
                liftDrive.set(-0.5);
            }

            @Override
            protected boolean isFinished() {
                return !bottomLimitSwitch.get();
            }
        };
    }

    public Command climbUp() {
        return new CommandChain("Climb Up").then(disengageBrake())
                .then(shiftToHighGear())
                .then(goToHeight(LiftHeights.kClimb, true))
                .then(shiftToLowGear())
                .then(goToHeight(LiftHeights.kScaleLow, false))
                .then(engageBrake());
    }

    public Command shiftToHighGear() {
        return new ActionCommand("Shift To High Gear", this, () -> {
            setGear(Gear.High);
        });
    }

    public Command shiftToLowGear() {
        return new ActionCommand("Shift To Low Gear", this, () -> {
            setGear(Gear.Low);
        });
    }

    private enum Gear {
        Low,
        High
    }

    private void setGear(final Gear gear) {
        if (gear == Gear.Low) {
            liftTransmission.set(Value.kForward);
        } else {
            liftTransmission.set(Value.kReverse);
        }
    }

    public Command engageBrake() {
        return new ActionCommand("Brake", this, this::doEngageBrake);
    }

    private void doEngageBrake() {
        liftBrake.set(Value.kReverse);
    }

    public Command disengageBrake() {
        return new ActionCommand("Don't Brake", this, this::doDisengageBrake);
    }

    private void doDisengageBrake() {
        liftBrake.set(Value.kReverse);
    }
}
