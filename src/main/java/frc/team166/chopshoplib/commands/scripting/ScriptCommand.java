package frc.team166.chopshoplib.commands.scripting;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * A command that is parsed from a string script.
 */
public class ScriptCommand extends Command {

    String script;
    Engine engine;
    Command generatedCommand;

    public static final Engine DEFAULT_ENGINE = new SimpleEngine();

    /**
     * Create a command that parses the given script when it's started.
     * @param script The script string to evaluate
     */
    public ScriptCommand(String script) {
        this(script, DEFAULT_ENGINE);
    }

    /**
     * Create a command that parses the given script when it's started.
     * @param name The name of the command
     * @param script The script string to evaluate
     */
    public ScriptCommand(String name, String script) {
        this(name, script, DEFAULT_ENGINE);
        this.script = script;
    }

    /**
     * Create a command that parses the given script when it's started.
     * @param script The script string to evaluate
     * @param engine The scripting engine to use for this command
     */
    public ScriptCommand(String script, Engine engine) {
        this.script = script;
        this.engine = engine;
    }

    /**
     * Create a command that parses the given script when it's started.
     * @param name The name of the command
     * @param script The script string to evaluate
     * @param engine The scripting engine to use for this command
     */
    public ScriptCommand(String name, String script, Engine engine) {
        super(name);
        this.script = script;
        this.engine = engine;
    }

    /**
     * Create a command that parses and runs a ScriptCommand from a preference.
     * @param key The preference name to read from
     * @param engine The scripting engine to use
     */
    public static Command fromPreference(String key, Engine engine) {
        Preferences prefs = Preferences.getInstance();
        if (!prefs.containsKey(key)) {
            prefs.putString(key, "");
        }
        return new InstantCommand(key) {
            @Override
            protected void initialize() {
                String script = Preferences.getInstance().getString(key, "");
                Command cmd = new ScriptCommand(key, script, engine);
                cmd.start();
            }
        };
    }

    /**
     * Create a command that parses and runs a ScriptCommand from a preference.
     * @param key The preference name to read from
     */
    public static Command fromPreference(String key) {
        return fromPreference(key, DEFAULT_ENGINE);
    }

    /**
     * Register a command function with the given prefix.
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command, given a double parameter
     */
    public static void registerHandler(String prefix, Function<String, Command> func) {
        DEFAULT_ENGINE.registerHandler(prefix, func);
    }

    /**
     * Register a command function with the given prefix.
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command, given a double parameter
     */
    public static void register(String prefix, Function<Double, Command> func) {
        DEFAULT_ENGINE.register(prefix, func);
    }

    /**
     * Register a command function with the given prefix.
     * @param prefix The prefix for use in scripts
     * @param func The function that creates the given command
     */
    public static void register(String prefix, Supplier<Command> func) {
        DEFAULT_ENGINE.register(prefix, func);
    }

    /**
     * Register a scriptable object.
     * 
     * <p>Calls the object's registerHandler method
     * @param s The object to use in scripts
     */
    public static void register(Scriptable s) {
        DEFAULT_ENGINE.register(s);
    }

    /**
     * Unregister a command function with the given prefix.
     *
     * <p>If no new command is specified for this prefix, its usage in scripts will be an error
     * @param prefix The prefix for use in scripts
     */
    public static void unregister(String prefix) {
        DEFAULT_ENGINE.unregister(prefix);
    }

    @Override
    protected void initialize() {
        generatedCommand = engine.parseScript(script);
        generatedCommand.start();
    }

    @Override
    protected boolean isFinished() {
        if (generatedCommand == null) {
            return false;
        }
        return generatedCommand.isCanceled() || generatedCommand.isCompleted();
    }

}
