package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

public class AlgaeSequence extends SequentialCommandGroup {
    /** Creates a new L4Sequence. */
    public AlgaeSequence(Arm m_arm, AlgaeIntake m_AlgaeIntake, Elevator m_elevator, DriveSubsystem m_drive) {
        addCommands(
                // Sets elevator and arm to processor position
                new SetArmAndElevatorPositions(m_elevator, m_arm, 0.026, 0.361),
                new InstantCommand(()->m_drive.drive(1, 0, 0, false)).withTimeout(.5),
                new OutakeAlgae(m_AlgaeIntake, 1.1));
    }
}
