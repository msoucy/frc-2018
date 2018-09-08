package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.team166.chopshoplib.DashboardUtils;
import frc.team166.chopshoplib.Display;
import frc.team166.chopshoplib.Resettable;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.outputs.SendableSpeedController;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;

public final class Manipulator extends PIDSubsystem implements Resettable {

    private final SendableSpeedController deploymentMotor;
    private final SpeedControllerGroup rollers;
    private final DoubleSolenoid innerSolenoid;
    private final DoubleSolenoid outerSolenoid;
    private final AnalogInput irSensor;
    private final AnalogPotentiometer potentiometer;

    private static final double P = 0.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 0.0;

    public Manipulator(final RobotMap map) {
        super("Manipulator", P, I, D, F);

        setAbsoluteTolerance(5);

        deploymentMotor = map.getDeploymentMotor();
        rollers = map.getRollers();
        innerSolenoid = map.getInnerManipSolenoid();
        outerSolenoid = map.getOuterManipSolenoid();
        irSensor = map.getManipIrSensor();
        potentiometer = map.getManipPotentiometer();

        DashboardUtils.initialize(this);

        deploymentMotor.setInverted(true);
    }

    // METHODS
    @Override
    public void reset() {
        rollers.stopMotor();
        deploymentMotor.stopMotor();
    }

    private void openInnerManipulator() {
        innerSolenoid.set(Value.kForward);
    }

    private void closeInnerManipulator() {
        innerSolenoid.set(Value.kReverse);
    }

    private void openOuterManipulator() {
        outerSolenoid.set(Value.kForward);
    }

    private void closeOuterManipulator() {
        outerSolenoid.set(Value.kReverse);
    }

    private double getIRDistance() {
        return irSensor.getVoltage();
    }

    @Override
    protected double returnPIDInput() {
        return potentiometer.pidGet();
    }

    @Override
    protected void usePIDOutput(final double output) {
        deploymentMotor.set(output);
    }

    /**
     * Sets motors to intake mode
     *
     * <p>
     * Turns motors on to intake a cube
     */
    private void setMotorsToIntake() {
        // change once you find optimal motor speed
        rollers.set(-0.6);
    }

    /**
     * Sets motors to discharge mode
     *
     * <p>
     * Turns motors on to discharge a cube
     */
    private void setMotorsToDischarge() {
        // change once you find optimal motor speed
        rollers.set(0.6);
    }

    // COMMANDS
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(DefaultCommand());
    };

    public Command DefaultCommand() {
        return new ActionCommand("Manipulator Default Command", this, () -> {
            DeployManipulatorWithJoystick(Robot.xBoxTempest).start();
        });
    }

    @Display
    public Command CloseOuterManipulator() {
        return new ActionCommand("Close Outer Manipulator", this, this::closeOuterManipulator);
    }

    @Display
    public Command OpenOuterManipulator() {
        return new ActionCommand("Open Outer Manipulator", this, this::openOuterManipulator);
    }

    @Display
    public Command OpenInnerManipulator() {
        return new ActionCommand("Open Inner Manipulator", this, this::openInnerManipulator);
    }

    @Display
    public Command CloseInnerManipulator() {
        return new ActionCommand("Close Inner Manipulator", this, this::closeInnerManipulator);
    }

    @Display
    public Command ManipulatorDischarge() {
        return new ActionCommand("DisCharge Manipulator", this, this::setMotorsToDischarge);
    }

    @Display
    public Command ManipulatorIntake() {
        return new ActionCommand("Intake Manipulator", this, this::setMotorsToIntake);
    }

    public Command ManipulatorIntakeHeld() {
        return new SubsystemCommand("Intake", this) {

            @Override
            protected void initialize() {
                setMotorsToIntake();
            }

            @Override
            protected boolean isFinished() {
                return false;

            }

            @Override
            protected void end() {
                rollers.set(0);
            }
        };
    }

    public Command ManipulatorDischargeHeld() {
        return new SubsystemCommand("Discharge", this) {

            @Override
            protected void initialize() {
                setMotorsToDischarge();
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                rollers.set(0);
            }
        };
    }

    public Command CubeEject() {
        return new SubsystemCommand("Eject Cube", this) {

            @Override
            protected void initialize() {
                setTimeout(5.0);
                final String gameData = DriverStation.getInstance()
                        .getGameSpecificMessage();

                if (gameData.length() > 0 && gameData.charAt(0) == 'R') {
                    setMotorsToDischarge();
                }
            }

            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }

            @Override
            protected void end() {
                rollers.stopMotor();
            }
        };
    }

    public Command CubePickup() {
        return new SubsystemCommand("Pick Up Cube", this) {

            @Override
            protected void initialize() {
                openOuterManipulator();
                setMotorsToIntake();
            }

            @Override
            protected boolean isFinished() {
                return getIRDistance() > 0.5;
            }

            @Override
            protected void end() {
                closeOuterManipulator();
                rollers.stopMotor();
            }
        };
    }

    public Command DeployManipulator() {
        return new SubsystemCommand("Deploy Manipulator", this) {

            @Override
            protected void initialize() {
                setSetpoint(2.5);
            }

            @Override
            protected boolean isFinished() {
                return onTarget();
            }
        };
    }

    public Command DeployManipulatorWithJoystick(final XboxController controller) {
        return new SubsystemCommand("Deploy Manipulator With Joystick", this) {
            @Override
            protected void initialize() {
                disable();
            }

            @Override
            protected void execute() {
                double rotation = Math.pow(controller.getY(Hand.kLeft), 2);
                rotation *= Math.signum(controller.getY(Hand.kLeft));
                deploymentMotor.set(rotation);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

        };
    }

    public Command enablePID() {
        return new ActionCommand("Enable PID", this, this::enable);
    }

}
