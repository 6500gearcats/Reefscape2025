package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class AlgaeSequence extends SequentialCommandGroup {
    /** Creates a new L4Sequence. */
    public AlgaeSequence(Arm m_arm, AlgaeIntake m_AlgaeIntake, Elevator m_elevator) {
        addCommands(
                // Sets elevator and arm to processor position
                new SetArmAndElevatorPositions(m_elevator, m_arm, 0.026, 0.361),
                new OutakeAlgae(m_AlgaeIntake, 1.1));
    }
}
