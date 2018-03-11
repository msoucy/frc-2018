package frc.team166.chopshoplib.outputs;

import edu.wpi.first.wpilibj.DigitalOutput;

public class DigitalOutputDutyCycle extends DigitalOutput {

    private double m_rate = 0;

    public DigitalOutputDutyCycle(final int channel) {
        super(channel);
    }

    @Override
    public void setPWMRate(double rate) {
        m_rate = rate;
        super.setPWMRate(rate);
    }

    @Override
    public void enablePWM(double initialDutyCycle) {
        m_rate = initialDutyCycle;
        super.enablePWM(initialDutyCycle);
    }

    public double getPWMRate() {
        return m_rate;
    }
}