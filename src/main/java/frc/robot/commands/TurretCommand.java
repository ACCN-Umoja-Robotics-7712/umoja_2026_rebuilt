package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class TurretCommand extends Command {
    ShooterTurretSubsystem turret;
    Supplier<Double> turretSpeed;

    public TurretCommand(ShooterTurretSubsystem turret, Supplier<Double> turretSpeed){
        this.turret = turret;
        this.turretSpeed = turretSpeed;

        addRequirements(turret); 
    }

    @Override
    public void initialize(){
        System.out.println("Turret Initialized");
    }

    @Override
    public void execute(){
        turret.runTurret(turretSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Turret end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
    