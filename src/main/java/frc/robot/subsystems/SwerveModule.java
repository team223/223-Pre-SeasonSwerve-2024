package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private double lastAngle;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  public SwerveModuleState desiredState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    setDesiredState(desiredState, isOpenLoop, false);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideSpeedLimit) {
    
    desiredState =
        OnboardModuleState.optimize(
            desiredState,
            getState().angle);
            
            // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not

    this.desiredState = desiredState;
    //If no PID is in use, set the motor's output. Otherwise, PID stays in effect
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
    /*double angle = desiredState.angle.getDegrees(); 
    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = angle;*/
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) || overrideSpeedLimit
            ? lastAngle
            : desiredState.angle.getDegrees(); 
    
    //PID deadband (stop when close). When within two degrees of desired angle, PID disables
    if (Math.abs(getState().angle.getDegrees() - angle) > 2) {
      angleController.setReference(angle, ControlType.kPosition);
      //SmartDashboard.putBoolean("Mod " + moduleNumber + " PID Active", true);
      lastAngle = angle;
    } else {
      angleMotor.set(0);
      //SmartDashboard.putBoolean("Mod " + moduleNumber + " PID Active", false);
    }
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset;
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  //Settings stuff
  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
  }

  //Settings stuff
  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.drivePositionConversionFactor);
    driveController.setP(Constants.Swerve.driveKP);
    driveController.setI(Constants.Swerve.driveKI);
    driveController.setD(Constants.Swerve.driveKD);
    driveController.setFF(Constants.Swerve.driveKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public void resetDriveEncoder() {
    driveEncoder.setPosition(0.0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d angle = Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    return new SwerveModuleState(velocity, angle);
  }

  public SwerveModulePosition getPosition() {
    double position = driveEncoder.getPosition();
    Rotation2d angle = Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    return new SwerveModulePosition(position, angle);
  }
}
