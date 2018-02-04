package frc.team166.chopshoplib.commands.scripting;

/**
 * An autonomous that is parsed from a string script
 */
@FunctionalInterface
public interface Scriptable {

    void registerScriptable(Engine e);

}
