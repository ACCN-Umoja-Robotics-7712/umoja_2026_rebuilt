package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterStates;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class ShooterTurretAngleCommand extends Command {
    ShooterTurretSubsystem turret;
    Supplier<Double> wantedTurretAngle;

    public ShooterTurretAngleCommand(ShooterTurretSubsystem turret, Supplier<Double> wantedTurretAngle){
        this.turret = turret;
        this.wantedTurretAngle = wantedTurretAngle;

        addRequirements(turret); 
    }

    @Override
    public void initialize(){
        System.out.println("Turret Initialized");
    }

    @Override
    public void execute(){
        turret.setTurretAngle(wantedTurretAngle.get());
    }

    @Override
    public void end(boolean isInterrupted){
        turret.runTurret(0);
        System.out.println("Turret end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
    