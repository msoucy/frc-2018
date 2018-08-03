/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating aroun
 */
public class RobotMap {
    public static class CAN {
        // changes motor ports into integers
        public final static int FRONT_RIGHT = 4;
        public final static int FRONT_LEFT = 8;
        public final static int BACK_RIGHT = 5;
        public final static int BACK_LEFT = 9;
        public final static int ROLLER_LEFT = 3;
        public final static int ROLLER_RIGHT = 2;
        public final static int LIFT_MOTOR_A = 6;
        public final static int LIFT_MOTOR_B = 7;
        public final static int DEPLOYMENT_MOTOR = 1;
    }

    public static class AnalogInputs {
        // changes input ports into integers
        public final static int tempestgyro = 1;
        public final static int IR = 2;
        public final static int MANIPULATOR_POTENTIOMETER = 3;
    }

    public static class Buttons {
        // changes button ports into integers
        public final static int XboxAbutton = 1;
        public final static int XboxBbutton = 2;
        public final static int XboxXbutton = 3;
        public final static int JoystickTrigger = 1;
    }

    public static class Solenoids {
        // changes Solenoid ports into integers

        public final static int LIFT_TRANSMISSION_A = 4;
        public final static int LIFT_TRANSMISSION_B = 5;
        public final static int MANIPULATOR_SOLENOID_INNER_A = 3;
        public final static int MANIPULATOR_SOLENOID_INNER_B = 2;
        public final static int MANIPULATOR_SOLENOID_OUTER_A = 1;
        public final static int MANIPULATOR_SOLENOID_OUTER_B = 0;
        public final static int LIFT_BRAKE_A = 7;
        public final static int LIFT_BRAKE_B = 6;
    }

    public static class DigitalInputs {
        // changes digital imput ports into integers
        public final static int LIFT_LIMIT_SWITCH_BOTTOM = 9;
        public final static int LIFT_LIMIT_SWITCH_TOP = 8;
        public final static int RED_LED = 4;
        public final static int GREEN_LED = 5;
        public final static int BLUE_LED = 6;
        public final static int LIFT_A = 0;
        public final static int LIFT_B = 1;
    }
}