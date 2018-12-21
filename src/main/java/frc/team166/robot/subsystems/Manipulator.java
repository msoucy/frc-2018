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
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import com.chopshop166.chopshoplib.Display;
import com.chopshop166.chopshoplib.Resettable;
import com.chopshop166.chopshoplib.commands.SetCommand;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
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

    public enum State {
        CLOSED,
        OPEN;
    }

    public enum MotorState {
        INTAKE(-0.6),
        STOPPED(0.0),
        DISCHARGE(0.6);

        private double value;

        MotorState(final double value) {
            this.value = value;
        }

        double getValue() {
            return value;
        }
    }

    public Manipulator(final RobotMap map) {
        super("Manipulator", P, I, D, F);

        setAbsoluteTolerance(5);

        deploymentMotor = map.getDeploymentMotor();
        rollers = map.getRollers();
        innerSolenoid = map.getInnerManipSolenoid();
        outerSolenoid = map.getOuterManipSolenoid();
        irSensor = map.getManipIrSensor();
        potentiometer = map.getManipPotentiometer();

        deploymentMotor.setInverted(true);
    }

    // METHODS
    @Override
    public void reset() {
        rollers.stopMotor();
        deploymentMotor.stopMotor();
    }

    private void setInner(final State state) {
        if (state == State.OPEN) {
            innerSolenoid.set(Value.kForward);
        } else {
            innerSolenoid.set(Value.kReverse);
        }
    }

    private void setOuter(final State state) {
        if (state == State.OPEN) {
            outerSolenoid.set(Value.kForward);
        } else {
            outerSolenoid.set(Value.kReverse);
        }
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

    private void setMotors(final MotorState state) {
        rollers.set(state.getValue());
    }

    // COMMANDS
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(DeployManipulatorWithJoystick(Robot.COPILOT));
    };

    @Display
    public Command OpenOuterManipulator() {
        return new SetCommand<>("Open Outer Manipulator", this, State.OPEN, this::setOuter);
    }

    @Display
    public Command CloseOuterManipulator() {
        return new SetCommand<>("Close Outer", this, State.CLOSED, this::setOuter);
    }

    @Display
    public Command OpenInnerManipulator() {
        return new SetCommand<>("Open Inner", this, State.OPEN, this::setInner);
    }

    @Display
    public Command CloseInnerManipulator() {
        return new SetCommand<>("Close Inner", this, State.CLOSED, this::setInner);
    }

    @Display
    public Command ManipulatorDischarge() {
        return new SetCommand<>("DisCharge", this, MotorState.DISCHARGE, this::setMotors);
    }

    @Display
    public Command ManipulatorIntake() {
        return new SetCommand<>("Intake", this, MotorState.INTAKE, this::setMotors);
    }

    public Command ManipulatorIntakeHeld() {
        return new Command("Intake", this) {

            @Override
            protected void initialize() {
                setMotors(MotorState.INTAKE);
            }

            @Override
            protected boolean isFinished() {
                return false;

            }

            @Override
            protected void end() {
                setMotors(MotorState.STOPPED);
            }
        };
    }

    public Command ManipulatorDischargeHeld() {
        return new Command("Discharge", this) {

            @Override
            protected void initialize() {
                setMotors(MotorState.DISCHARGE);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                setMotors(MotorState.STOPPED);
            }
        };
    }

    public Command CubeEject() {
        return new Command("Eject Cube", this) {

            @Override
            protected void initialize() {
                setTimeout(5.0);
                final String gameData = DriverStation.getInstance()
                        .getGameSpecificMessage();

                if (gameData.length() > 0 && gameData.charAt(0) == 'R') {
                    setMotors(MotorState.DISCHARGE);
                }
            }

            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }

            @Override
            protected void end() {
                setMotors(MotorState.STOPPED);
            }
        };
    }

    public Command CubePickup() {
        return new Command("Pick Up Cube", this) {

            @Override
            protected void initialize() {
                setOuter(State.OPEN);
                setMotors(MotorState.INTAKE);
            }

            @Override
            protected boolean isFinished() {
                return getIRDistance() > 0.5;
            }

            @Override
            protected void end() {
                setOuter(State.CLOSED);
                setMotors(MotorState.STOPPED);
            }
        };
    }

    public Command DeployManipulator() {
        return new Command("Deploy Manipulator", this) {

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
        return new Command("Deploy Manipulator With Joystick", this) {
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
        return new InstantCommand("Enable PID", this, this::enable);
    }

}
