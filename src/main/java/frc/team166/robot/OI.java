/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

import frc.team166.chopshoplib.controls.ButtonJoystick;
import frc.team166.chopshoplib.controls.ButtonXboxController;

public class OI {
    // Creates joysticks
    public ButtonJoystick leftDriveStick;
    public ButtonJoystick rightDriveStick;
    public ButtonXboxController xboxTempest;

    /**
     * Initialize buttons and joysticks.
     */
    public OI() {
        //defines the joysticks as joysticks and assigns left and right
        leftDriveStick = new ButtonJoystick(RobotMap.Controller.leftcontrol);
        rightDriveStick = new ButtonJoystick(RobotMap.Controller.rightcontrol);
        leftDriveStick.getButton(RobotMap.Buttons.JoystickTrigger).whileHeld(Robot.drive.driveStraight());
        xboxTempest = new ButtonXboxController(RobotMap.Controller.Xboxcontrol);
        // xBoxTempest.getButton(RobotMap.Buttons.XboxAbutton).whileHeld(Robot.drive.Ebrake());
        xboxTempest.getButton(RobotMap.Buttons.XboxXbutton).whileHeld(Robot.drive.driveStraight());
    }
}
