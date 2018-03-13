package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder.BooleanConsumer;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.robot.RobotMap;

public class LED extends Subsystem {

    public void initDefaultCommand() {
    }

    //these will be changed from DigitalOutputs to something else when we get real hardware...
    DigitalOutput red = new DigitalOutput(RobotMap.DigitalInputs.RED_LED);
    DigitalOutput green = new DigitalOutput(RobotMap.DigitalInputs.GREEN_LED);
    DigitalOutput blue = new DigitalOutput(RobotMap.DigitalInputs.BLUE_LED);

    // METHODS
    private void allOff() {
        red.set(false);
        green.set(false);
        blue.set(false);
    }

    private boolean isBlueTeam() {
        Alliance team = DriverStation.getInstance().getAlliance();
        return (team == DriverStation.Alliance.Blue);
    }

    private void setTeamColor(boolean turnOn) {
        if (isBlueTeam()) {
            red.set(false);
            blue.set(turnOn);
        } else {
            blue.set(false);
            red.set(turnOn);
        }
    }

    // COMMANDS
    /**
     * Blink the green light.
     * @param numberOfBlinks The number of times to blink the light
     */
    public Command blinkGreen(int numberOfBlinks) {
        return new SubsystemCommand(this) {
            double lastUpdateTime = System.currentTimeMillis();
            boolean isOn = true;
            double count = 0;

            @Override
            protected void initialize() {
                count = 0;
                green.set(true);
            }

            @Override
            protected void execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 250) {
                    lastUpdateTime = System.currentTimeMillis();
                    if (isOn) {
                        green.set(false);
                        isOn = false;
                        count++;
                    } else {
                        green.set(true);
                        isOn = true;

                    }
                }
            }

            @Override
            protected boolean isFinished() {
                if (count >= numberOfBlinks) {
                    return true;
                } else {
                    return false;
                }
            }

            @Override
            protected void end() {
                green.set(false);
            }
        };
    }

    /**
     * Blink the team color.
     */
    public Command blinkTeamColor() {
        return new SubsystemCommand(this) {
            double lastUpdateTime = System.currentTimeMillis();
            boolean isOn = true;

            @Override
            protected void initialize() {
                setTeamColor(true);
            }

            @Override
            protected void execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 750) {
                    lastUpdateTime = System.currentTimeMillis();
                    isOn = !isOn;
                    setTeamColor(isOn);
                }
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                setTeamColor(false);
            }
        };
    }

    public Command blueOn() {
        return onOff(blue::set);
    }

    /**
     * Set cyan (blue + green) on.
     */
    public Command cyanOn() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                red.set(false);
                blue.set(true);
                green.set(true);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                allOff();
            }
        };
    }

    public Command greenOn() {
        return onOff(green::set);
    }

    public Command lightTeamColor() {
        return onOff(this::setTeamColor);
    }

    public Command redOn() {
        return onOff(red::set);
    }

    private Command onOff(BooleanConsumer light) {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                light.accept(true);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                light.accept(false);
            }
        };
    }

}
