package frc.robot.commands.drive.autos;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveDistanceMeters;

public class Autocone3 extends SequentialCommandGroup {
    public Autocone3(Drivetrain s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber,Timer m_timer){

        Command driveback = new DriveDistanceMeters(s_Swerve, -3.5, .4); 

        addCommands(
            ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
            driveback
            //GroundPickup.GroundPickupCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
            //traj2,
            //ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber)
        );
    }
}