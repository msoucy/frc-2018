package frc.team166.chopshoplib;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;

public interface AutoChildren {
    default public void addChildren(Subsystem system) {
        Class<? extends Subsystem> aClass = system.getClass();
        for (Field field : aClass.getDeclaredFields()) {
            try {
                // See if the returned object implements sendable.
                // If it does then lets add it as a child.
                if (Sendable.class.isAssignableFrom(field.getType())) {
                    system.addChild(field.getName(), (Sendable) field.get(system));
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }

        }
    }
}