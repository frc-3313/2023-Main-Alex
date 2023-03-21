package frc.robot.commands.drive.autos;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.TrajectoryHelper;
import frc.robot.commands.drive.FollowHolonomicTrajectory;

public class BlueLeft2Piece extends SequentialCommandGroup {
    public BlueLeft2Piece(Swerve s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber,Timer m_timer){

        Command waitcommand = new WaitCommand(1);
        Command traj1 = new FollowHolonomicTrajectory(s_Swerve, TrajectoryHelper.loadJSONTrajectory("BlueLeft1_Out.wpilib.json"), true);
       Command traj2 = new FollowHolonomicTrajectory(s_Swerve, TrajectoryHelper.loadJSONTrajectory("BlueLeft1_Back.wpilib.json"), false);


        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand),
            ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber)
            //traj1,
            //GroundPickup.GroundPickupCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
            //traj2,
            //ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber)
        );
    }
}