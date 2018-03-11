package frc.team166.chopshoplib.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Commands {
    /**
     * Repeat a command a given number of times.
     * @param n The number of times to run the command
     * @param c The command to run
     */
    public static Command repeat(final int n, Command c) {
        class RepeatedCommand extends Command {
            int numTimesRun = 0;
            Command cmd = c;

            @Override
            protected void initialize() {
                numTimesRun++;
                cmd.start();
            }

            @Override
            protected void execute() {
                if (cmd.isCompleted()) {
                    numTimesRun++;
                    cmd.start();
                }
            }

            @Override
            protected boolean isFinished() {
                return numTimesRun >= n;
            }

            @Override
            protected void end() {
                numTimesRun = 0;
            }
        }

        return new RepeatedCommand();
    }

    /**
     * Repeat a command a given number of times.
     * @param n The number of times to run the command
     * @param c A function creating the command to run
     */
    public static Command repeat(final int n, Supplier<Command> c) {
        class RepeatedCommand extends CommandGroup {

            public RepeatedCommand() {
                for (int i = 0; i < n; i++) {
                    addSequential(c.get());
                }
            }
        }

        return new RepeatedCommand();
    }

    /**
     * Repeat a command a given number of times.
     * @param cond The condition to test against
     * @param c The command to run
     */
    public static Command repeatWhile(BooleanSupplier cond, Command c) {
        class RepeatedCommand extends Command {
            Command cmd = c;
            boolean shouldFinish = false;

            @Override
            protected void execute() {
                if (!cmd.isRunning()) {
                    if (cond.getAsBoolean()) {
                        cmd.start();
                    } else {
                        shouldFinish = true;
                    }
                }
            }

            @Override
            protected boolean isFinished() {
                return shouldFinish;
            }
        }

        return new RepeatedCommand();
    }

    /**
     * Create a command from a Runnable.
     * @param func The action to perform
     */
    public static Command from(Runnable func) {
        return new ActionCommand(func);
    }

    /**
     * Start a CommandChain.
     * @param cs The first actions to perform in the chain
     */
    public static Command first(Command... cs) {
        return new CommandChain(cs);
    }
}