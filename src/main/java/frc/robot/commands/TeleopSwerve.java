package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private IntSupplier povSup;

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier xPatternSup,
      IntSupplier povSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.povSup = povSup;
  }

  @Override
  public void execute() {
    
    /* Get Values, Deadband*/
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal =
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal =
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    if (povSup.getAsInt() != -1) {
      s_Swerve.fineDrive(360 - povSup.getAsInt());
    } else {

      SmartDashboard.putNumber("Speed", translationVal);
    
      /* Drive */
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
          rotationVal * Constants.Swerve.maxAngularVelocity,
          !robotCentricSup.getAsBoolean(),
          true);
    }

    SmartDashboard.putNumber("Joystick", Math.toDegrees(Math.atan2(translationVal, strafeVal)));
    
  }
}
