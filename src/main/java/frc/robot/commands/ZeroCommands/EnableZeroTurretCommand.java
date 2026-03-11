package frc.robot.commands.ZeroCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class EnableZeroTurretCommand extends Command {
    
    ShooterTurretSubsystem turretSubsystem;

    public EnableZeroTurretCommand(ShooterTurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void initialize(){
        System.out.println("Starting zeroing hood");
        this.turretSubsystem.enableZero();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Zeroing hood finished inturrupted: " + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
