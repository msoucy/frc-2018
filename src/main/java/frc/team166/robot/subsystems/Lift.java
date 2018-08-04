package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;

public class Lift extends PIDSubsystem {
    // This is for one inch
    private final double encoderDistancePerTick = 0.01636;

    DigitalInput bottomLimitSwitch;
    DigitalInput topLimitSwitch;
    Encoder Encoder;
    SpeedController liftDrive;
    DoubleSolenoid liftBrake;
    DoubleSolenoid liftTransmission;

    // TODO we need to calculate these
    // these define the PID values for the lift
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kF = 0;

    // this adds the LIDAR sensor
    private Lidar liftLidar = new Lidar(Port.kOnboard, 0x60);

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

        LiftHeights(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }

    // sets the maximum lidar distance before switching to the encoder
    private final static double kMaxLidarDistance = 60;

    public Lift(RobotMap.LiftMap map) {
        super("Lift", kP, kI, kD, kF);
        bottomLimitSwitch = map.getLiftBottomLimit();
        topLimitSwitch = map.getLiftTopLimit();
        Encoder = map.getLiftEncoder();
        liftDrive = map.getLiftMotors();

        liftBrake = map.getLiftBrake();
        liftTransmission = map.getLiftShifter();

        setOutputRange(-1, 1);
        setAbsoluteTolerance(0.05);
        Encoder.setDistancePerPulse(encoderDistancePerTick);
        // creates a child for the encoders and other stuff
        // (limit switches, lidar, etc.)
        addChild("Encoder", Encoder);
        addChild("Top", topLimitSwitch);
        addChild("Bottom", bottomLimitSwitch);
        addChild("LiDAR", liftLidar);
        addChild("Transmission", liftTransmission);
        addChild("Brake", liftBrake);
        // addChild("Drive", liftDrive);
        addChild(findLiftHeight());

        liftDrive.setInverted(true);

        if (!Preferences.getInstance()
                .containsKey("Use LIDAR")) {
            Preferences.getInstance()
                    .putBoolean("Use LIDAR", false);
        }

        registerCommands();
    }

    private void registerCommands() {
        // SmartDashboard.putData("Brake", Brake());
        // SmartDashboard.putData("Shift to Low Gear", ShiftToLowGear());
        // SmartDashboard.putData("Shift to High Gear", ShiftToHighGear());
        SmartDashboard.putData("Go up distance", MoveLiftByInches(8));
    }

    protected double returnPIDInput() {
        return findLiftHeight();
    }

    private void raiseLift() {
        liftDrive.set(-0.75);
    }

    private void lowerLift() {
        liftDrive.set(-0.5);
    }

    private void engageBrake() {
        liftBrake.set(Value.kForward);
    }

    private void disengageBrake() {
        liftBrake.set(Value.kReverse);
    }

    protected void usePIDOutput(double output) {
        if (topLimitSwitch.get() == false && output > 0) {
            setSetpoint(LiftHeights.kMaxHeight.get());
            liftDrive.stopMotor();
            return;
        }
        if (bottomLimitSwitch.get() == false && output < 0) {
            // liftEncoder.reset();
            setSetpoint(LiftHeights.kFloor.get());
            liftDrive.stopMotor();
            return;
        }
        liftDrive.set(output);
    }

    public void reset() {
        liftDrive.stopMotor();
    }

    public double findLiftHeight() {
        if (Preferences.getInstance()
                .getBoolean("Use LIDAR", false) == true) {
            if (liftLidar.getDistance(true) > kMaxLidarDistance) {
                return (liftLidar.getDistance(true));
            } else {
                return (Encoder.getDistance());
            }
        } else {
            return (Encoder.getDistance());
        }
    }

    // gear changes
    public void shiftToHighGear() {
        liftTransmission.set(Value.kReverse);
    }

    public void shiftToLowGear() {
        liftTransmission.set(Value.kForward);
    }

    // does not do anything
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(ManualLift(Robot.xBoxTempest));
    }

    public Command RaiseLiftALittle() {
        return new SubsystemCommand("Raise Lift A Little", this) {
            @Override
            protected void initialize() {
                setTimeout(2.5);
                disengageBrake();
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
                engageBrake();
                liftDrive.set(0);
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    public Command GoToHeight(LiftHeights height, boolean isHighGear) {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disengageBrake();
                if (isHighGear == true) {
                    shiftToHighGear();
                } else {
                    shiftToLowGear();
                }
                setSetpoint(height.get());
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
        };
    }

    public Command ManualLift(final XboxController controller) {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                // disengageBrake();
                disable();
            }

            @Override
            protected void execute() {
                double elevatorControl = controller.getTriggerAxis(Hand.kRight)
                        - controller.getTriggerAxis(Hand.kLeft);

                if (elevatorControl >= .1 || elevatorControl <= -0.1) {
                    disengageBrake();
                } else {
                    engageBrake();

                }
                if (elevatorControl > 0 && !topLimitSwitch.get()) {
                    liftDrive.set(controller.getTriggerAxis(Hand.kLeft));
                } else if ((elevatorControl < 0 && !bottomLimitSwitch.get())) {
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

            @Override
            protected void interrupted() {
                enable();
            }
        };
    }

    public Command MoveLiftByInches(double inches) {
        return new SubsystemCommand(this) {
            double LiftDestinationHeight;

            @Override
            protected void initialize() {
                disable();
                disengageBrake();
                LiftDestinationHeight = Encoder.getDistance() + inches;
                if (LiftDestinationHeight > Encoder.getDistance()) {
                    liftDrive.set(0.75);
                } else {
                    liftDrive.set(-0.50);
                }
            }

            @Override
            protected boolean isFinished() {
                if (liftDrive.get() > 0) {
                    if (Encoder.getDistance() >= LiftDestinationHeight) {
                        return true;
                    }
                } else {
                    if (Encoder.getDistance() <= LiftDestinationHeight) {
                        return true;
                    }
                }
                return false;
            }

            @Override
            protected void end() {
                liftDrive.set(0);
                engageBrake();
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    public Command GoUp() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disengageBrake();
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

    public Command GoDown() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disengageBrake();
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

    public Command LowerLiftToLimitSwitch() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                disengageBrake();
            }

            @Override
            protected void execute() {
                lowerLift();
            }

            @Override
            protected boolean isFinished() {
                return !bottomLimitSwitch.get();
            }
        };
    }

    public Command ClimbUp() {
        return new CommandChain("Climb Up").then(DisengageBrake())
                .then(ShiftToHighGear())
                .then(GoToHeight(LiftHeights.kClimb, true))
                .then(ShiftToLowGear())
                .then(GoToHeight(LiftHeights.kScaleLow, false))
                .then(Brake());
    }

    public Command ShiftToHighGear() {
        return new ActionCommand("Shift To High Gear", this, this::shiftToHighGear);
    }

    public Command ShiftToLowGear() {
        return new ActionCommand("Shift To Low Gear", this, this::shiftToLowGear);
    }

    public Command Brake() {
        return new ActionCommand("Brake", this, this::engageBrake);
    }

    public Command DisengageBrake() {
        return new ActionCommand("Don't Brake", this, this::disengageBrake);
    }
}
