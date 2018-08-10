package frc.team166.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.ActionCommand;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.robot.RobotMap;

public final class LED extends Subsystem {
    
    DigitalOutputDutyCycle red;
    DigitalOutputDutyCycle green;
    DigitalOutputDutyCycle blue;

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(BreathTeamColor());
    }

    public LED(RobotMap map) {
        super();
        registerCommands();

        red = map.getRedLED();
        addChild("Red", red);

        green = map.getGreenLED();
        addChild("Green", green);

        blue = map.getBlueLED();
        addChild("Blue", blue);
    }

    // METHODS
    void registerCommands() {
        SmartDashboard.putData("Breath Blue", Breath(blue, 10));
        SmartDashboard.putData("All Off", new ActionCommand("OFF GERALD", this, this::allOff));
        SmartDashboard.putData("STEVIE WONDER THEM", NotSeizure(1000));
        SmartDashboard.putData("NYAN", Rainbow(1000));
    }

    void allOff() {
        red.set(false);
        green.set(false);
        blue.set(false);
    }

    boolean isBlueTeam() {
        Alliance team = DriverStation.getInstance()
                .getAlliance();
        return (team == DriverStation.Alliance.Blue);
    }

    void setTeamColor(boolean turnOn) {
        if (isBlueTeam()) {
            red.set(false);
            blue.set(turnOn);
        } else {
            blue.set(false);
            red.set(turnOn);
        }
    }

    // COMMANDS
    public Command BlinkGreen(int numberOfBlinks) {
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
                return (count >= numberOfBlinks);
            }

            @Override
            protected void end() {
                green.set(false);
            }
        };
    }

    public Command BlinkTeamColor() {
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

    Command ColorOn(DigitalOutputDutyCycle color) {
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

    public Command RedOn() {
        return ColorOn(red);
    }

    public Command GreenOn() {
        return ColorOn(green);
    }

    public Command BlueOn() {
        return ColorOn(blue);
    }

    public Command CyanOn() {
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

    public Command LightTeamColor() {
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

    public Command Breath(DigitalOutputDutyCycle color, int frequency) {
        return new SubsystemCommand("fade", this) {
            boolean isDutyCycleIncreasing = true;
            double period;
            // Approx how often execute is called
            static final double executePeriod = 20 * 0.001;
            static final double dutyCycleChangePerPeriod = 2.0;
            double changeAmount;

            @Override
            protected void initialize() {
                period = (1.0 / frequency);
                changeAmount = dutyCycleChangePerPeriod / ((period / executePeriod));
                color.enablePWM(0);
                isDutyCycleIncreasing = true;
            }

            @Override
            protected void execute() {
                if (isDutyCycleIncreasing) {
                    color.updateDutyCycle(color.getPWMRate() + changeAmount);
                } else {
                    color.updateDutyCycle(color.getPWMRate() - changeAmount);
                }
                if ((color.getPWMRate() >= 1) || (color.getPWMRate() <= 0)) {
                    isDutyCycleIncreasing = !isDutyCycleIncreasing;
                }

            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };
    }

    Command BreathTeamColor() {
        return new ActionCommand("Breath Team Color", this, () -> {
            if (isBlueTeam()) {
                red.disablePWM();
                green.disablePWM();
                Breath(blue, 2).start();

            } else {
                blue.disablePWM();
                green.disablePWM();
                Breath(red, 2).start();

            }
        });
    }

    public Command NotSeizure(int numberOfBlinks) {

        return new SubsystemCommand(this) {
            double lastUpdateTime = System.currentTimeMillis();
            boolean isOn = true;
            double count = 0;

            @Override
            protected void initialize() {
                count = 0;
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
                return (count >= numberOfBlinks);
            }

            @Override
            protected void end() {
                red.set(false);
                blue.set(false);
            }
        };
    }

    public Command Rainbow(int numberOfBlinks) {

        return new SubsystemCommand(this) {
            double lastUpdateTime = System.currentTimeMillis();
            Random r = new Random();
            double count = 0;

            @Override
            protected void initialize() {
                count = 0;
                blue.set(true);
            }

            @Override
            protected void execute() {
                if (System.currentTimeMillis() >= lastUpdateTime + 15) {
                    lastUpdateTime = System.currentTimeMillis();
                    blue.set(r.nextBoolean());
                    red.set(r.nextBoolean());
                    green.set(r.nextBoolean());
                    count++;
                }
            }

            @Override
            protected boolean isFinished() {
                return (count >= numberOfBlinks);
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
