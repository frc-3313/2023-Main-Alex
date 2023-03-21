package frc.robot.commands.drive.autos;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.TrajectoryHelper;
import frc.robot.commands.drive.FollowHolonomicTrajectory;
import frc.robot.commands.drive.AutoBalanceSimple;

public class BlueCenterBalance extends SequentialCommandGroup {
    public BlueCenterBalance(Swerve s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber){
        //Command dropPiececommand = new AutoOpenGrabber(s_Grabber);

        Command waitcommand = new WaitCommand(1);
//        Command traj1 = new FollowHolonomicTrajectory(s_Swerve, TrajectoryHelper.loadJSONTrajectory("BlueChargeGrid5ToPiece3.wpilib.json"), true);
//        Command traj2 = new FollowHolonomicTrajectory(s_Swerve, TrajectoryHelper.loadJSONTrajectory("BlueRight1_Back.wpilib.json"), false);


        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand),
            ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber)
        //    traj1,
        //    GroundPickup.GroundPickupCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
         //   traj2,
        //    ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
        //    new AutoBalanceSimple(s_Swerve)

        );

    }
}