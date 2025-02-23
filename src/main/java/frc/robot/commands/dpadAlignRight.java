package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

public class dpadAlignRight extends dpadAlign{

    public dpadAlignRight(DriveSubsystem drive) {
        super(drive, true);
    }
}
