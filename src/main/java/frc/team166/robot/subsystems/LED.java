package frc.team166.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.DashboardUtils;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.DefaultDashboard;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.robot.RobotMap;

public final class LED extends Subsystem {

    private final DigitalOutputDutyCycle red;
    private final DigitalOutputDutyCycle green;
    private final DigitalOutputDutyCycle blue;

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(breathTeamColor());
    }

    public LED(final RobotMap.LEDMap map) {
        super();

        red = map.getRed();
        green = map.getGreen();
        blue = map.getBlue();

        SmartDashboard.putData("All Off", new ActionCommand("OFF GERALD", this, this::allOff));

        DashboardUtils.initialize(this);
    }

    // METHODS
    private void allOff() {
        red.set(false);
        green.set(false);
        blue.set(false);
    }

    private boolean isBlueTeam() {
        final Alliance team = DriverStation.getInstance()
                .getAlliance();
        return team == DriverStation.Alliance.Blue;
    }

    private void setTeamColor(final boolean turnOn) {
        if (isBlueTeam()) {
            red.set(false);
            blue.set(turnOn);
        } else {
            blue.set(false);
            red.set(turnOn);
        }
    }

    // COMMANDS
    public Command blinkGreen(final int numberOfBlinks) {
        return new SubsystemCommand(this) {
            private double lastUpdateTime = System.currentTimeMillis();
            private boolean isOn = true;
            private double count;

            @Override
            protected void initialize() {
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
                return count >= numberOfBlinks;
            }

            @Override
            protected void end() {
                green.set(false);
            }
        };
    }

    public Command blinkTeamColor() {
        return new SubsystemCommand(this) {
            private double lastUpdateTime = System.currentTimeMillis();
            private boolean isOn = true;

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

    private Command colorOn(final DigitalOutputDutyCycle color) {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                color.set(true);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                color.set(false);
            }
        };
    }

    public Command redOn() {
        return colorOn(red);
    }

    public Command greenOn() {
        return colorOn(green);
    }

    public Command blueOn() {
        return colorOn(blue);
    }

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

    public Command lightTeamColor() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                setTeamColor(true);
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

    public Command breath(final DigitalOutputDutyCycle color, final int frequency) {
        return new SubsystemCommand("fade", this) {
            // Approx how often execute is called
            private static final double EXEC_PERIOD = 20 * 0.001;
            private static final double DUTY_CYCLE_CHANGE = 2.0;
            private boolean isIncreasing = true;
            private final double period = 1.0 / frequency;
            private final double changeAmount = DUTY_CYCLE_CHANGE / (period / EXEC_PERIOD);

            @Override
            protected void initialize() {
                color.enablePWM(0);
            }

            @Override
            protected void execute() {
                if (isIncreasing) {
                    color.updateDutyCycle(color.getPWMRate() + changeAmount);
                } else {
                    color.updateDutyCycle(color.getPWMRate() - changeAmount);
                }
                if (color.getPWMRate() >= 1 || color.getPWMRate() <= 0) {
                    isIncreasing = !isIncreasing;
                }

            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    @Display
    public Command breathTeamColor() {
        return new ActionCommand("Breath Team Color", this, () -> {
            green.disablePWM();
            if (isBlueTeam()) {
                red.disablePWM();
                breath(blue, 2).start();
            } else {
                blue.disablePWM();
                breath(red, 2).start();

            }
        });
    }

    @Display(1000)
    public Command notSeizure(final int numberOfBlinks) {
        return new SubsystemCommand(this) {
            private double lastUpdateTime = System.currentTimeMillis();
            private boolean isOn = true;
            private double count;

            @Override
            protected void initialize() {
                blue.set(true);
            }

            @Override
            protected void execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 15) {
                    lastUpdateTime = System.currentTimeMillis();
                    if (isOn) {
                        blue.set(false);
                        red.set(true);
                        isOn = false;
                        count++;
                    } else {
                        blue.set(true);
                        red.set(false);
                        isOn = true;
                    }
                }
            }

            @Override
            protected boolean isFinished() {
                return count >= numberOfBlinks;
            }

            @Override
            protected void end() {
                red.set(false);
                blue.set(false);
            }
        };
    }

    @Display(1000)
    public Command rainbow(final int numberOfBlinks) {
        return new SubsystemCommand(this) {
            private double lastUpdateTime = System.currentTimeMillis();
            private final Random rand = new Random();
            private double count;

            @Override
            protected void initialize() {
                blue.set(true);
            }

            @Override
            protected void execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 15) {
                    lastUpdateTime = System.currentTimeMillis();
                    blue.set(rand.nextBoolean());
                    red.set(rand.nextBoolean());
                    green.set(rand.nextBoolean());
                    count++;
                }
            }

            @Override
            protected boolean isFinished() {
                return count >= numberOfBlinks;
            }

            @Override
            protected void end() {
                red.set(false);
                blue.set(false);
                green.set(false);
            }
        };
    }
}
