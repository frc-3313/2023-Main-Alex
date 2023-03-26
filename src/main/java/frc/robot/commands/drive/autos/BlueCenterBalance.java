package frc.robot.commands.drive.autos;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueCenterBalance extends SequentialCommandGroup {
    public BlueCenterBalance(Drivetrain s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber){
        //Command dropPiececommand = new AutoOpenGrabber(s_Grabber);

        addCommands(
            ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber)
        //    traj1,
        //    GroundPickup.GroundPickupCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
         //   traj2,
        //    ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
        //    new AutoBalanceSimple(s_Swerve)

        );

    }
}