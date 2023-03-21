package frc.robot.commands.drive.autos;

import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class MeterPlace extends SequentialCommandGroup {
    public MeterPlace(Swerve s_Swerve, Arm s_Arm,Wrist s_Wrist, Grabber s_Grabber, Timer m_timer){

        Command setArmHigh = new ArmWristCommand(s_Arm, Constants.HIGH_ARM_ANGLE, Constants.MAX_ARM_SPEED, s_Wrist, Constants.HIGH_WRIST_ANGLE, Constants.MAX_WRIST_SPEED);
        Command setArmStow = new ArmWristCommand(s_Arm, Constants.STOW_ARM_ANGLE, Constants.MAX_ARM_SPEED, s_Wrist, Constants.STOW_WRIST_ANGLE, Constants.MAX_WRIST_SPEED);
        Command dropPiececommand = new AutoOpenGrabber(s_Grabber);

        Command waitcommand = new WaitCommand(1);
        Command as1command = new DriveDistanceMeters(s_Swerve, 1, .5);
        Command as2command = new DriveDistanceMeters(s_Swerve, -1, .5);


        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand),
            setArmHigh.withTimeout(3),
            as1command,
            new WaitCommand(.25),
            dropPiececommand.withTimeout(.5),
            as2command,
            setArmStow.withTimeout(3)
        );
    }
}