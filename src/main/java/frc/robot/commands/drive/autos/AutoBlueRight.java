package frc.robot.commands.drive.autos;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.Strafe;
import frc.robot.commands.drive.DriveDistanceMeters;

public class AutoBlueRight extends SequentialCommandGroup {
    public AutoBlueRight(Drivetrain s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber,Timer m_timer){

        //Command dropPiececommand = new AutoOpenGrabber(s_Grabber);

        Command strafe = new Strafe(s_Swerve, -.2, .5);
        Command driveback = new DriveDistanceMeters(s_Swerve, -4, .7);
        Command driveback2 = new DriveDistanceMeters(s_Swerve, -.1, .1);
        Command waitcommand2 = new WaitCommand(.5);
        Command waitcommand3 = new WaitCommand(.5);
        Command waitcommand4 = new WaitCommand(.1);


        addCommands(
            ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
            waitcommand2,
            strafe,
            waitcommand3,
            driveback2,
            waitcommand4,
            driveback
        //    traj1,
        //    GroundPickup.GroundPickupCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
        //    traj2,
        //    ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber)
        );
    }
}