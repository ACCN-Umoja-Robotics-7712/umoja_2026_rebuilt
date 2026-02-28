package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class SetShooterFlywheelStateCommand extends Command{
    ShooterFlywheelSubsystem flywheelSubsystem;
    double flywheelState;

    public SetShooterFlywheelStateCommand(ShooterFlywheelSubsystem flywheelSubsystem, double flywheelState){
        this.flywheelSubsystem = flywheelSubsystem;
        this.flywheelState = flywheelState;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("flywheel state START: " + flywheelState);
        flywheelSubsystem.setState(flywheelState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("flywheel state END INTERRUPTED: " + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return flywheelSubsystem.didReachVelocity();
    }
}