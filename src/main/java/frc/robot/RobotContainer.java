package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.drive.autos.Autocone3;
import frc.robot.commands.drive.autos.Autoleftred;
import frc.robot.commands.drive.autos.AutoBlueRight;
import frc.robot.commands.drive.autos.AutoPoseTesting;
import frc.robot.commands.ArmWristCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    SendableChooser<Command> auto_chooser = new SendableChooser<>();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final ButtonBox m_buttonBox = new ButtonBox(1);

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Grabber m_grabber = new Grabber();
    private final Arm m_arm = new Arm();
    private final Wrist m_wrist = new Wrist();
    public final Timer m_timer = new Timer();
    public double gyroOffset = 0.0;

    /* Autos */
    private final Command auto1 = new Autocone3(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);
    private final Command auto2 = new Autoleftred(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);
    private final Command auto3 = new AutoBlueRight(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);
    private final Command testing = new AutoPoseTesting(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {     
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(Constants.LEFTY)/1.3, 
                () -> -driver.getRawAxis(Constants.LEFTX)/1.3, 
                () -> driver.getRawAxis(Constants.RIGHTX)/2/9*13.5,    ///2/9*6.75, 
                () -> robotCentric.getAsBoolean(), 
                () -> driver.getRawAxis(Constants.RIGHTTRIGGER)/1.5,
                () -> gyroOffset
               // () -> driver.getRawButton(Constants.START)
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        
        auto_chooser.setDefaultOption("Middle field", auto1);
        auto_chooser.addOption("edge Red", auto2);
        auto_chooser.addOption("edge Blue", auto3);
        //auto_chooser.addOption("don't choose this", testing);
        SmartDashboard.putData(auto_chooser);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> this.zeroGyro()));
    // ARM (CONTROL STICK)
    m_buttonBox.RightBumper.whileTrue(new RunCommand(() -> m_wrist.raise())).onFalse(new InstantCommand(() -> m_wrist.stop()));
    m_buttonBox.LeftBumper.whileTrue(new RunCommand(() -> m_wrist.lower())).onFalse(new InstantCommand(() -> m_wrist.stop()));
    m_buttonBox.RightTrigger.whileTrue(new RunCommand(() -> m_arm.raise())).onFalse(new InstantCommand(() -> m_arm.stop()));
    m_buttonBox.LeftTrigger.whileTrue(new RunCommand(() -> m_arm.lower())).onFalse(new InstantCommand(() -> m_arm.stop()));
    // SET LOW
    m_buttonBox.setLow.onTrue(new ArmWristCommand(m_arm, Constants.LOW_ARM_ANGLE, Constants.MAX_ARM_SPEED, m_wrist, Constants.LOW_WRIST_ANGLE, Constants.MAX_WRIST_SPEED));

    // SET MID
    m_buttonBox.setMiddle.onTrue(new ArmWristCommand(m_arm, Constants.MID_ARM_ANGLE, Constants.MAX_ARM_SPEED, m_wrist, Constants.MID_WRIST_ANGLE, Constants.MAX_WRIST_SPEED));

    // SET HIGH
    m_buttonBox.setHigh.onTrue(new ArmWristCommand(m_arm, Constants.HIGH_ARM_ANGLE, Constants.MAX_ARM_SPEED, m_wrist, Constants.HIGH_WRIST_ANGLE, Constants.MAX_WRIST_SPEED));
    //SET SHELF
    m_buttonBox.RightDpadTrigger.onTrue(new ArmWristCommand(m_arm, Constants.SHELF_ARM_ANGLE, Constants.MAX_ARM_SPEED, m_wrist, Constants.SHELF_WRIST_ANGLE, Constants.MAX_WRIST_SPEED));
    // SET stow
    m_buttonBox.setStow.onTrue(new ArmWristCommand(m_arm, Constants.STOW_ARM_ANGLE, Constants.MAX_ARM_SPEED, m_wrist, Constants.STOW_WRIST_ANGLE, Constants.MAX_WRIST_SPEED));
    

    //grabber commands  
    m_buttonBox.LeftJoystick.onTrue(m_grabber.grabPieceFactory());
    m_buttonBox.RightJoystick.onTrue(m_grabber.dropPieceFactory());
    m_buttonBox.DownDpadTrigger.onTrue(m_grabber.startRollersCommand()).onFalse(m_grabber.stopRollersCommand());
    m_buttonBox.UpDpadTrigger.onTrue(m_grabber.startRollersReverseCommand()).onFalse(m_grabber.stopRollersCommand());

    }

    public void zeroGyro() {
        s_Swerve.zeroGyro();
        gyroOffset = 0.0;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return auto1;
        return auto_chooser.getSelected();
        //return auto1;
    }
}
