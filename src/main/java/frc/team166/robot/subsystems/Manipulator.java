package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;

public class Manipulator extends PIDSubsystem {

    WPI_VictorSPX deploymentMotor = new WPI_VictorSPX(RobotMap.CAN.DEPLOYMENT_MOTOR);

    WPI_VictorSPX leftRoller = new WPI_VictorSPX(RobotMap.CAN.ROLLER_LEFT);
    WPI_TalonSRX rightRoller = new WPI_TalonSRX(RobotMap.CAN.ROLLER_RIGHT);

    SpeedControllerGroup rollers = new SpeedControllerGroup(leftRoller, rightRoller);

    DoubleSolenoid innerSolenoid = new DoubleSolenoid(RobotMap.Solenoids.MANIPULATOR_SOLENOID_INNER_A,
            RobotMap.Solenoids.MANIPULATOR_SOLENOID_INNER_B);

    DoubleSolenoid outerSolenoid = new DoubleSolenoid(RobotMap.Solenoids.MANIPULATOR_SOLENOID_OUTER_A,
            RobotMap.Solenoids.MANIPULATOR_SOLENOID_OUTER_B);

    AnalogInput irSensor = new AnalogInput(RobotMap.AnalogInputs.IR);

    AnalogPotentiometer potentiometer = new AnalogPotentiometer(RobotMap.AnalogInputs.MANIPULATOR_POTENTIOMETER);

    // inches:
    final static double ROLLER_RADIUS = 1.4375;
    // ft:
    final static double DIST_PER_PULSE_INTAKE = (((ROLLER_RADIUS * 2.0 * Math.PI) / 1024.0) / 12.0);

    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kF = 0;

    public Manipulator() {
        super("Manipulator", kP, kI, kD, kF);

        setAbsoluteTolerance(5);

        addChild(rollers);
        addChild("IR", irSensor);
        addChild("Potentiometer", potentiometer);
        addChild("Deploy Motor", deploymentMotor);
        addChild("Rollers", rollers);
        addChild("Inner", innerSolenoid);
        addChild("Outer", outerSolenoid);

        leftRoller.setInverted(false);
        rightRoller.setInverted(true);
        deploymentMotor.setInverted(true);

        // Adding Commands To SmartDashboard
        // SmartDashboard.putData("Close Outer", CloseOuterManipulator());
        // SmartDashboard.putData("Open Outer", OpenOuterManipulator());
        // SmartDashboard.putData("Close Inner", CloseInnerManipulator());
        // SmartDashboard.putData("Open Inner", OpenInnerManipulator());
        // SmartDashboard.putData("Cube Eject", CubeEject());
        // SmartDashboard.putData("Cube Pickup", CubePickup());
        // SmartDashboard.putData("cube pickup with lights", CubePickupWithLights(3));
        // SmartDashboard.putData("Deploy Manipulator With Joystick",
        // DeployManipulatorWithJoystick());
        // SmartDashboard.putData("Re-Enable Potentiometer", enablePID());
    }

    // METHODS
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
    protected void usePIDOutput(double output) {
        deploymentMotor.set(output);
    }

    /**
     * Sets motors to intake mode
     * <p>
     * Turns motors on to intake a cube
     */
    private void setMotorsToIntake() {
        // change once you find optimal motor speed
        rollers.set(-0.6);
    }

    /**
     * Sets motors to discharge mode
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

    public Command CloseOuterManipulator() {
        return new ActionCommand("Close Outer Manipulator", this, this::closeOuterManipulator);
    }

    public Command OpenOuterManipulator() {
        return new ActionCommand("Open Outer Manipulator", this, this::openOuterManipulator);
    }

    public Command OpenInnerManipulator() {
        return new ActionCommand("Open Inner Manipulator", this, this::openInnerManipulator);
    }

    public Command CloseInnerManipulator() {
        return new ActionCommand("Close Inner Manipulator", this, this::closeInnerManipulator);
    }

    public Command ManipulatorDischarge() {
        return new ActionCommand("DisCharge Manipulator", this, this::setMotorsToDischarge);
    }

    public Command ManipulatorIntake() {
        return new ActionCommand("Intake Manipulator", this, this::setMotorsToIntake);
    }

    public Command CubeDrop() {
        return new ActionCommand("Drop Cube", this, this::openInnerManipulator);
    }

    public Command CubeClamp() {
        return new ActionCommand("Cube Clamp", this, this::closeInnerManipulator);
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

            @Override
            protected void interrupted() {
                end();
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

            @Override
            protected void interrupted() {
                end();
            }

        };
    }

    public Command CubeEject() {
        return new SubsystemCommand("Eject Cube", this) {

            @Override
            protected void initialize() {
                setTimeout(5.0);
                String gameData;
                gameData = DriverStation.getInstance().getGameSpecificMessage();

                if (gameData.length() > 0) {
                    if (gameData.charAt(0) == 'R') {
                        setMotorsToDischarge();
                    }
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

            @Override
            protected void interrupted() {
                end();
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
                return (getIRDistance() > 0.5);
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
        return new SubsystemCommand("Deploy Manipulator With Joystick") {
            @Override
            protected void initialize() {
                disable();
            }

            @Override
            protected void execute() {
                double rotation = Math.pow(controller.getY(Hand.kLeft), 2);

                rotation *= (controller.getY(Hand.kLeft) / Math.abs(controller.getY(Hand.kLeft)));

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