package frc.robot.commands.autos;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.*;

public class Drive1Meter extends SequentialCommandGroup {
    public Drive1Meter(Swerve s_Swerve, Timer m_timer){



        Command waitcommand = new WaitCommand(1);
        //Command stopswervecommand = new AutoSpeed(s_Swerve, 0, 0, 0, 0, m_timer, false);
        Command as1command = new DriveDistanceMetersNew(s_Swerve, 1, .5);
        

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand),
            as1command
            //,
            //stopswervecommand

        );
    }
}