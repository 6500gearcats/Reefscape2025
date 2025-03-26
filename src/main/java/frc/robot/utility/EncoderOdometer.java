package frc.robot.utility;

import com.revrobotics.RelativeEncoder;

public class EncoderOdometer {

    private RelativeEncoder m_encoder;
    
    private double m_startPosition;


    public EncoderOdometer(RelativeEncoder encoder) {
        m_encoder = encoder;
        m_startPosition = m_encoder.getPosition();
    }

    public double getPosition() {
        double position=m_encoder.getPosition();
        return position - m_startPosition;
    }

    public void reset() {
        m_startPosition = m_encoder.getPosition();
    }

    public RelativeEncoder getEncoder() {
        return m_encoder;
    }
    
}