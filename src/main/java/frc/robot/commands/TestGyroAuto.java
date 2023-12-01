package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class TestGyroAuto extends Command{
    private Swerve s_Swerve;
    private double x;
    private double y;
    private double rotation;
    public TestGyroAuto (Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        
    }

    @Override
    public void initialize() {
        s_Swerve.zeroGyro();
        s_Swerve.resetOdometry(new Pose2d());
        x = s_Swerve.swerveOdometry.getPoseMeters().getX();
        y = s_Swerve.swerveOdometry.getPoseMeters().getY();
        rotation = s_Swerve.swerveOdometry.getPoseMeters().getRotation().getDegrees();

    }
    
    @Override
    public void execute() {
        s_Swerve.updateOdometry();
        x = s_Swerve.swerveOdometry.getPoseMeters().getX();
        y = s_Swerve.swerveOdometry.getPoseMeters().getY();
        rotation = s_Swerve.swerveOdometry.getPoseMeters().getRotation().getDegrees();
        //(s_Swerve.getModulePositions()[0].distanceMeters + s_Swerve.getModulePositions()[1].distanceMeters + s_Swerve.getModulePositions()[2].distanceMeters + s_Swerve.getModulePositions()[3].distanceMeters)/4;
        
        if(rotation < 30 && x < 2){
          System.out.println(rotation);
          System.out.println(x);
          s_Swerve.drive(new Translation2d(0.5,0).times(Constants.Swerve.maxSpeed), 0.5, false, false);

        }else{
          System.out.println("stop");
          s_Swerve.drive(new Translation2d(), 0, false, false);
          
        }
        
        
    }
    @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(), 0, false, false);

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

  





