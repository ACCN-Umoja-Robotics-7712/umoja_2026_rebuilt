package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class IntakeRollerCommand extends Command{
    ShooterTurretSubsystem turret;
    Supplier<Double> turretSpeed;

    public IntakeRollerCommand(ShooterTurretSubsystem turret, Supplier<Double> turretSpeed){
        this.turret = turret;
        this.turretSpeed = turretSpeed;

        addRequirements(turret);
    }

    @Override
    public void initialize(){
        System.out.println("Intake Rollers Initialized");
    }

    @Override
    public void execute(){
        turret.runTurret(turretSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Intake Rollers end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
