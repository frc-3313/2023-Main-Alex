// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

 package frc.robot.commands.drive;

 import edu.wpi.first.wpilibj.DriverStation.Alliance;
 import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj2.command.ConditionalCommand;
 import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
 import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmWristCommand;
import frc.robot.commands.drive.autos.AutoOpenGrabber;
import frc.robot.subsystems.Swerve;
 import frc.robot.subsystems.Arm;
 import frc.robot.subsystems.Wrist;
 import frc.robot.subsystems.Grabber;
 import frc.robot.util.TrajectoryHelper;
 import frc.robot.Constants;

// /** Add your docs here. */
 public class Autonomous {

     public static ConditionalCommand engage(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new ConditionalCommand(redEngage(drive, arm, wrist, grabber), 
             blueEngage(drive, arm, wrist, grabber),
             () -> {return DriverStation.getAlliance() == Alliance.Red;});
     }

     public static ConditionalCommand center2(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new ConditionalCommand(center2("Red", drive, arm, wrist, grabber), 
             center2("Blue", drive, arm, wrist, grabber),
             () -> {return DriverStation.getAlliance() == Alliance.Red;});
     }

     public static ConditionalCommand center2Balance(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new ConditionalCommand(center2Balance("Red", drive, arm, wrist, grabber), 
             center2Balance("Blue", drive, arm, wrist, grabber),
             () -> {return DriverStation.getAlliance() == Alliance.Red;});
     }

     public static ConditionalCommand center3(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new ConditionalCommand(center3("Red", drive, arm, wrist, grabber), 
             center3("Blue", drive, arm, wrist, grabber),
             () -> {return DriverStation.getAlliance() == Alliance.Red;});
     }

     public static ConditionalCommand center3Balance(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new ConditionalCommand(center3Balance("Red", drive, arm, wrist, grabber), 
             center3Balance("Blue", drive, arm, wrist, grabber),
             () -> {return DriverStation.getAlliance() == Alliance.Red;});
     }

     public static SequentialCommandGroup upAndOver(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
            initialScoreHigh(drive, arm, wrist, grabber),
            new ConditionalCommand(
                 new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngageOver.wpilib.json"), false),
                 new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("BlueEngageOver.wpilib.json"), false),
                 () -> {return DriverStation.getAlliance() == Alliance.Red;}),
            new WaitCommand(0.5),
            PickupGround(drive, arm, wrist, grabber),
            new ConditionalCommand(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngageBack.wpilib.json"), false),
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("BlueEngageBack.wpilib.json"), false),
                () -> {return DriverStation.getAlliance() == Alliance.Red;}),
            initialScoreMid(drive, arm, wrist, grabber),
            new AutoBalanceSimple(drive)
         );
     }

     public static SequentialCommandGroup redEngage(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
            initialScoreHigh(drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngage.wpilib.json"), false)
         );
     }

     public static SequentialCommandGroup blueEngage(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
            initialScoreHigh(drive, arm, wrist, grabber),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngage.wpilib.json"), false)
         );
     }

     public static SequentialCommandGroup center3Balance(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
             center3(alliance, drive, arm, wrist, grabber),
             centerBalance(alliance, drive, arm, wrist, grabber)
         );
     }

     public static SequentialCommandGroup center3(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
             centerBaseTo1Mid(alliance, drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), false),
             PickupGround(drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToGrid1.wpilib.json"), false),
             initialScoreMid(drive, arm, wrist, grabber));
     }

     public static SequentialCommandGroup center2Balance(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
            centerBaseTo1High(alliance, drive, arm, wrist, grabber),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), false),
            PickupGround(drive, arm, wrist, grabber),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToEngage.wpilib.json"), false),
            new AutoBalanceSimple(drive)
         );
     }

     public static SequentialCommandGroup center2(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return centerBaseTo1High(alliance, drive, arm, wrist, grabber);
     }

     private static SequentialCommandGroup centerBaseTo1High(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
             initialScoreHigh(drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false),
             PickupGround(drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid1.wpilib.json"), false),
             initialScoreHigh(drive, arm, wrist, grabber)
         );
     }

     private static SequentialCommandGroup centerBaseTo1Mid(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
             initialScoreMid(drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false),
             PickupGround(drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid1.wpilib.json"), false),
             initialScoreMid(drive, arm, wrist, grabber));
     }

     private static SequentialCommandGroup centerBaseTo3(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
             initialScoreHigh(drive, arm, wrist, grabber),
             new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false),
             new ParallelCommandGroup(
                 new SequentialCommandGroup(
                     new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid3-1.wpilib.json"), false),
                     new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid3-2.wpilib.json"), false)
                 ),
                 initialScoreHigh(drive, arm, wrist, grabber))
         );
     }

     private static SequentialCommandGroup centerBalance(String alliance, Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterEngage.wpilib.json"), false)
         );
     }

     private static SequentialCommandGroup initialScoreHigh(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {

        return new SequentialCommandGroup(
             new ArmWristCommand(arm, Constants.HIGH_ARM_ANGLE, Constants.AUTO_ARM_SPEED, wrist, Constants.HIGH_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(3),
             new DriveDistanceMeters(drive, 1, .5),
             new WaitCommand(.25),
             new AutoOpenGrabber(grabber).withTimeout(.5),
             new DriveDistanceMeters(drive, -.5, .5),
             new ArmWristCommand(arm, Constants.STOW_ARM_ANGLE, Constants.AUTO_ARM_SPEED, wrist, Constants.STOW_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(0.5)
         );
     }

     private static SequentialCommandGroup initialScoreMid(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
         return new SequentialCommandGroup(
            new ArmWristCommand(arm, Constants.MID_ARM_ANGLE, Constants.AUTO_ARM_SPEED, wrist, Constants.MID_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(0.5),
            new DriveDistanceMeters(drive, .75, .5),
            new WaitCommand(.25),
            grabber.dropPieceFactory().withTimeout(0.25),
            new DriveDistanceMeters(drive, -.75, .5),
            new ArmWristCommand(arm, Constants.STOW_ARM_ANGLE, Constants.AUTO_ARM_SPEED, wrist, Constants.STOW_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(0.5)
         );
     }
     private static SequentialCommandGroup PickupGround(Swerve drive, Arm arm, Wrist wrist, Grabber grabber) {
        return new SequentialCommandGroup(
            new ArmWristCommand(arm, Constants.LOW_ARM_ANGLE, Constants.AUTO_ARM_SPEED, wrist, Constants.LOW_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(0.5),
            grabber.startRollersCommand().withTimeout(.25),
            new DriveDistanceMeters(drive, .25, .5),
            grabber.grabPieceFactory().withTimeout(0.5),
            grabber.stopRollersCommand().withTimeout(.1),
            new ArmWristCommand(arm, Constants.STOW_ARM_ANGLE, Constants.AUTO_ARM_SPEED, wrist, Constants.STOW_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(0.5)
        );
    }
 }
