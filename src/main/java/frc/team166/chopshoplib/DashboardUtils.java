package frc.team166.chopshoplib;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.Display;

public class DashboardUtils {
    private DashboardUtils() {
    }

    public static void initialize(Subsystem system) {
        Class<? extends Subsystem> aClass = system.getClass();

        for (Field field : aClass.getDeclaredFields()) {
            // Make the field accessible, because apparently we're allowed to do that
            field.setAccessible(true);
            try {
                // See if the returned object implements sendable.
                // If it does then lets add it as a child.
                if (Sendable.class.isAssignableFrom(field.getType())) {
                    System.out.println("Adding field: " + field.getName());
                    system.addChild(field.getName(), (Sendable) field.get(system));
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        for (Method method : aClass.getDeclaredMethods()) {
            try {
                for (Display annotation : method.getAnnotationsByType(Display.class)) {
                    Double[] args = RobotUtils.toBoxed(annotation.value());
                    Command command = (Command) method.invoke(system, (Object[]) args);
                    if (command != null) {
                        System.out.println("Adding command: " + command);
                        SmartDashboard.putData(command.getName(), command);
                    }
                }
            } catch (InvocationTargetException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }
}