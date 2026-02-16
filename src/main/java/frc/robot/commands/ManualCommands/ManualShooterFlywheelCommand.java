package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ManualShooterFlywheelCommand extends Command{
        ShooterFlywheelSubsystem flywheel;
        Supplier<Double> flywheelSpeed;
    
        public ManualShooterFlywheelCommand(ShooterFlywheelSubsystem flywheel, Supplier<Double> flywheelSpeed){
            this.flywheel = flywheel;
            this.flywheelSpeed = flywheelSpeed;

        addRequirements(flywheel);
    }

    @Override
    public void initialize(){
        System.out.println("Manual Flywheel Shooter Initialized");
    }

    @Override
    public void execute(){
        flywheel.runShooter(flywheelSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        flywheel.runShooter(0);
        System.out.println("Manual Flywheel Shooter end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

    

