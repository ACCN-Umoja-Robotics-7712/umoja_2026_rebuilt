package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualExactSwerveCommand extends Command {
    SwerveSubsystem swerveSubsystem;
    Supplier<Double> forwardSpeed, sideSpeed, turningSpeed;

    public ManualExactSwerveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> forwardSpeed, Supplier<Double> sideSpeed, Supplier<Double> turningSpeed){
        this.swerveSubsystem = swerveSubsystem;
        this.forwardSpeed = forwardSpeed;
        this.sideSpeed = sideSpeed;
        this.turningSpeed = turningSpeed;

        addRequirements(swerveSubsystem);
    }  

    @Override
    public void initialize(){
        System.out.println("Manual Swerve Initialized");
    }

    @Override
    public void execute(){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardSpeed.get(), sideSpeed.get(), turningSpeed.get());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Swerve end interrupted:" + isInterrupted);
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
