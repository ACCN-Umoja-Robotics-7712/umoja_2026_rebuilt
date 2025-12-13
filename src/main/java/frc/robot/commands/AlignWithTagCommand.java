package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class AlignWithTagCommand extends Command{
    SwerveSubsystem swerveSubsystem;

    public AlignWithTagCommand(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Align with tag Initialized");
    }

    @Override
    public void execute(){
        boolean canSeeTarget = LimelightHelpers.getTV(Constants.LimelightConstants.tagName);
        if (canSeeTarget){
            double target_x = LimelightHelpers.getTX(Constants.LimelightConstants.tagName);
            // double target_y = LimelightHelpers.getTY(Constants.LimelightConstants.tagName);
            // double target_a = LimelightHelpers.getTA(Constants.LimelightConstants.tagName);
            // Pose3d targetPose3d = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LimelightConstants.tagName);
            // Double targetYaw = Math.toDegrees(targetPose3d.getRotation().getAngle());

            swerveSubsystem.alignWithTag(target_x);
        }
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Align with tag end is interrupted:" + isInterrupted);
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
        // return swerveSubsystem.hasCoralSensor();
    }
    
}
