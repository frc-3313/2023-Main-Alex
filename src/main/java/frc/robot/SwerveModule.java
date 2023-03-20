package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
//import frc.robot.Constants;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.CANSparkMaxUtil;
//import frc.lib.util.CANCoderUtil;
import com.ctre.phoenix.sensors.CANCoderConfiguration;


public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

    public CANCoderConfiguration swerveCanCoderConfig;

    private SlewRateLimiter filter = new SlewRateLimiter(0.02);
    
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        swerveCanCoderConfig = new CANCoderConfiguration();
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        driveController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            /*if (mDriveMotor.get() >= desiredState.speedMetersPerSecond){*/
                double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
                mDriveMotor.set(percentOutput);
           /* }
            else {*/
                //double percentOutput = filter.calculate (desiredState.speedMetersPerSecond);
                //mDriveMotor.set(percentOutput);
            //}
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                feedforward.calculate(desiredState.speedMetersPerSecond));
          }
    }

    private void setAngle(SwerveModuleState desiredState){
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset;
        integratedAngleEncoder.setPosition(absolutePosition);
        //integratedAngleEncoder.setPosition(0);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKFF);
        mAngleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mDriveMotor, Usage.kAll);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKFF);
        mDriveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }
    public double getAbsoluteEncoderRad() {
        double angle = angleEncoder.getPosition();
        angle *= 2.0 * Math.PI / 360.0;
        angle -= angleOffset;
        return angle;
    }
}