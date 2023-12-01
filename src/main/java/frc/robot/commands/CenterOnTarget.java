package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class CenterOnTarget extends Command{
    private Swerve s_Swerve;
    private double x;
    private double y;
    public CenterOnTarget (Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        
    }

    @Override
    public void initialize() {
        s_Swerve.zeroGyro();
        s_Swerve.resetOdometry(new Pose2d());
        x = s_Swerve.swerveOdometry.getPoseMeters().getX();
        y = s_Swerve.swerveOdometry.getPoseMeters().getY();
    }
    
    @Override
    public void execute() {
        s_Swerve.updateOdometry();
        //(s_Swerve.getModulePositions()[0].distanceMeters + s_Swerve.getModulePositions()[1].distanceMeters + s_Swerve.getModulePositions()[2].distanceMeters + s_Swerve.getModulePositions()[3].distanceMeters)/4;
        System.out.println(x);
        if(x < Units.feetToMeters(4)){
         
          s_Swerve.fineDrive(0);
        }
        
    }
    @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0.0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
        
        

    /*    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(),
            new Pose2d()
        );
        
     PathPlanner path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(0.0, 0.0
        //, 2 * Math.PI, 4 * Math.PI
        ),
        new GoalEndState(0.0, Rotation2d.fromDegrees(0))
     );  */  
    
    
}

  





