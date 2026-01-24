package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ManualShooterFlywheelCommand extends Command{
        ShooterFlywheelSubsystem Flywheel;
        Supplier<Double> FlywheelSpeed;
    
        public ManualShooterFlywheelCommand(ShooterFlywheelSubsystem Flywheel, Supplier<Double> FlywheelSpeed){
            this.Flywheel = Flywheel;
            this.FlywheelSpeed = FlywheelSpeed;

        addRequirements(Flywheel);
    }

    @Override
    public void initialize(){
        System.out.println("Manual Flywheel Shooter Initialized");
    }

    @Override
    public void execute(){
        Flywheel.runShooter(FlywheelSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Flywheel Shooter end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

    

