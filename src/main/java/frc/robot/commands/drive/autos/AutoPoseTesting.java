package frc.robot.commands.drive.autos;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveDistanceMeters;
import frc.robot.commands.drive.GoToPose;

public class AutoPoseTesting extends SequentialCommandGroup {
    final Pose2d my_pose1 = new Pose2d(0.3, 0.0, new Rotation2d(0.0));
    final Pose2d my_pose2 = new Pose2d(.3, .3, new Rotation2d(0.0));
    final Pose2d my_pose3 = new Pose2d(.3, 0.0, new Rotation2d(0.0));
    final Pose2d my_pose4 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0)));

    public AutoPoseTesting(Swerve s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber,Timer m_timer){
        
        Command waitcommand = new WaitCommand(1);
        Command Pose1 = new GoToPose(s_Swerve, my_pose1);
        Command Pose2 = new GoToPose(s_Swerve, my_pose1);
        Command Pose3 = new GoToPose(s_Swerve, my_pose1);
        Command Pose4 = new GoToPose(s_Swerve, my_pose1);
 

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand),
            Pose1,
            new WaitCommand(.5),
            Pose2,
            new WaitCommand(.5),
            Pose3,
            new WaitCommand(.5),
            Pose4
            //GroundPickup.GroundPickupCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber),
            //traj2,
            //ScoreHigh.ScoreHighCommand(s_Swerve, s_Arm, s_Wrist, s_Grabber)
        );
    }
}