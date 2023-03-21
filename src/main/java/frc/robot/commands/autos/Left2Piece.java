package frc.robot.commands.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ArmWristCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveDistanceMeters;
import frc.robot.util.TrajectoryHelper;
import frc.robot.commands.newDrive.FollowTrajectory;

public class Left2Piece extends SequentialCommandGroup {
    public Left2Piece(Swerve s_Swerve, Arm s_Arm, Wrist s_wrist, Grabber s_Grabber,Timer m_timer){

        Command setArmHigh = new ArmWristCommand(s_Arm, Constants.HIGH_ARM_ANGLE, Constants.MAX_ARM_SPEED, s_wrist, Constants.HIGH_WRIST_ANGLE, Constants.MAX_WRIST_SPEED);
        Command setArmStow = new ArmWristCommand(s_Arm, Constants.STOW_ARM_ANGLE, Constants.MAX_ARM_SPEED, s_wrist, Constants.STOW_WRIST_ANGLE, Constants.MAX_WRIST_SPEED);
        Command setArmLow = new ArmWristCommand(s_Arm, Constants.LOW_ARM_ANGLE, Constants.MAX_ARM_SPEED, s_wrist, Constants.LOW_WRIST_ANGLE, Constants.MAX_WRIST_SPEED);
        Command dropPiececommand = new AutoOpenGrabber(s_Grabber);

        Command waitcommand = new WaitCommand(1);
        Command as1command = new DriveDistanceMeters(s_Swerve, 1, .5);
        Command as2command = new DriveDistanceMeters(s_Swerve, -1, .5);
        Command traj1 = new FollowTrajectory(s_Swerve, TrajectoryHelper.loadJSONTrajectory("Blue1_1.wpilib.json"), true);
        Command traj2 = new FollowTrajectory(s_Swerve, TrajectoryHelper.loadJSONTrajectory("Blue1_2.wpilib.json"), false);


        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand),
            setArmHigh.withTimeout(3),
            as1command,
            new WaitCommand(.25),
            dropPiececommand.withTimeout(.5),
            as2command,
            setArmStow.withTimeout(3),
            traj1,
            setArmLow.withTimeout(.5),
            s_Grabber.startRollersCommand().withTimeout(.25),
            new DriveDistanceMeters(s_Swerve, .25, .5),
            s_Grabber.grabPieceFactory().withTimeout(0.5),
            s_Grabber.stopRollersCommand().withTimeout(.1),
            setArmStow,
            traj2,
            setArmHigh.withTimeout(3),
            as1command,
            new WaitCommand(.25),
            dropPiececommand.withTimeout(.5),
            as2command,
            setArmStow.withTimeout(3)
        );
    }
}