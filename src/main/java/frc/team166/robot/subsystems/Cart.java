/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.commands.scripting.Engine;
import frc.team166.chopshoplib.commands.scripting.Scriptable;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Cart extends Subsystem implements Scriptable {
    Victor motor = new Victor(1);
    DigitalInput sensorright = new DigitalInput(8);
    DigitalInput sensorleft = new DigitalInput(9);

    public Cart() {
        addChild(motor);
        addChild(sensorleft);
        addChild(sensorright);

        motor.setInverted(true);
    }

    private boolean sensorrighton() {
        return !sensorright.get();
    }

    private boolean sensorlefton() {
        return !sensorleft.get();
    }

    private void setspeed(double speed) {
        motor.set(speed);
    }

    public void initDefaultCommand() {

    }

    public Command moveleft() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                setspeed(0.4);
            }

            @Override
            protected boolean isFinished() {
                return sensorlefton();
            }

            @Override
            protected void end() {
                setspeed(0);
            }
        };
    }

    public Command moveright() {
        return new SubsystemCommand(this) {
            @Override
            protected void initialize() {
                setspeed(-0.2);
            }

            @Override
            protected boolean isFinished() {
                return sensorrighton();
            }

            @Override
            protected void end() {
                setspeed(0);
            }
        };
    }

    @Override
    public void registerHandler(Engine e) {
        e.register("left", this::moveleft);
        e.register("right", this::moveright);
    }
}
