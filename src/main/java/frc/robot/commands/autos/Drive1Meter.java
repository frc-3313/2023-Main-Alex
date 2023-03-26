package frc.robot.commands.autos;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.*;

public class Drive1Meter extends SequentialCommandGroup {
    public Drive1Meter(Drivetrain s_Swerve, Timer m_timer){

        Command as1command = new DriveDistanceMetersNew(s_Swerve, 1, .5);
        

        addCommands(
            as1command
            //,
            //stopswervecommand

        );
    }
}