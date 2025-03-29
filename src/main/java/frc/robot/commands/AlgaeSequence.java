package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

public class AlgaeSequence extends SequentialCommandGroup {
    /** Creates a new L4Sequence. */
    public AlgaeSequence(Arm m_arm, AlgaeIntake m_AlgaeIntake, Elevator m_elevator, DriveSubsystem m_drive) {
        addCommands(
                // This code assumes AlgaeGrab has just run and the elevator/arm are already in correct positions
                // Outakes algae
                // TODO: Add an isFinished into OutakeAlgae so that we don't have to use withTimeout
                new OutakeAlgae(m_AlgaeIntake, 1.1).withTimeout(0.3));
    }
}
