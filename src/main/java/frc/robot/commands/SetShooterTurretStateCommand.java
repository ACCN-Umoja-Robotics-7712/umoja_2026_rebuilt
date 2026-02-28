package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class SetShooterTurretStateCommand extends Command{
    ShooterTurretSubsystem turret;
    double turretState;

    public SetShooterTurretStateCommand(ShooterTurretSubsystem turret, double turretState){
        this.turret = turret;
        this.turretState = turretState;
        addRequirements(turret);
    }

    @Override
    public void initialize(){
        System.out.println("turret state START: " + turretState);
        turret.setState(turretState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("turret state END INTERRUPTED: " + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return turret.didReachAngle();
    }
}