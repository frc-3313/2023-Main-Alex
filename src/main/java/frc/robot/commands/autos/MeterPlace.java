package frc.robot.commands.autos;

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

        Command setArmHigh = new ArmWristCommand(s_Arm, Constants.HIGH_ARM_ANGLE, s_Wrist, Constants.HIGH_WRIST_ANGLE);
        Command setArmStow = new ArmWristCommand(s_Arm, Constants.STOW_ARM_ANGLE, s_Wrist, Constants.STOW_WRIST_ANGLE);
        Command dropPiececommand = new AutoOpenGrabber(s_Grabber, 0.2, m_timer);

        Command waitcommand = new WaitCommand(1);
        //Command stopswervecommand = new AutoSpeed(s_Swerve, 0, 0, 0, 0, m_timer, false);
        Command as1command = new DriveDistanceMetersNew(s_Swerve, 1, .5);
        Command as2command = new DriveDistanceMetersNew(s_Swerve, -1, .5);


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