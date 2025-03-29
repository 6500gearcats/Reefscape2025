package frc.robot.utility;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ElevatorConstants;

public class EncoderOdometer {

    private RelativeEncoder m_encoder;
    
    private double m_startPosition;


    public EncoderOdometer(RelativeEncoder encoder) {
        m_encoder = encoder;
        m_startPosition = m_encoder.getPosition();
    }

    public double getPosition() {
        double position=m_encoder.getPosition();
        return (position - m_startPosition) * ElevatorConstants.kRotationsToMeters;
    }

    public void reset() {
        m_startPosition = m_encoder.getPosition();
    }

    public RelativeEncoder getEncoder() {
        return m_encoder;
    }

    public void setHeight(double elevatorHeight) {
        if (elevatorHeight > 0 ) {
            m_startPosition = m_encoder.getPosition() - elevatorHeight/ElevatorConstants.kRotationsToMeters;
        }
    }
    
}