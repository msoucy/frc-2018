package frc.team166.robot.maps;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.team166.chopshoplib.outputs.DigitalOutputDutyCycle;
import frc.team166.robot.RobotMap;

public class Tempest implements RobotMap {

    DigitalOutputDutyCycle m_redLED = new DigitalOutputDutyCycle(4);
    DigitalOutputDutyCycle m_greenLED = new DigitalOutputDutyCycle(5);
    DigitalOutputDutyCycle m_blueLED = new DigitalOutputDutyCycle(6);
    SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(new WPI_VictorSPX(8), new WPI_VictorSPX(9));
    SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(new WPI_VictorSPX(4), new WPI_VictorSPX(5));

    @Override
    public DigitalOutputDutyCycle getRedLED() {
        return m_redLED;
    }

    @Override
    public DigitalOutputDutyCycle getGreenLED() {
        return m_greenLED;
    }

    @Override
    public DigitalOutputDutyCycle getBlueLED() {
        return m_blueLED;
    }

    @Override
    public SpeedController getLeftWheelMotors() {
        return m_leftGroup;
    }

    @Override
    public SpeedController getRightWheelMotors() {
        return m_rightGroup;
    }

}