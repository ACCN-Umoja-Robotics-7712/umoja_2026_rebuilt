package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class SetShooterHoodStateCommand extends Command{
    ShooterHoodSubsystem hood;
    double hoodState;

    public SetShooterHoodStateCommand(ShooterHoodSubsystem hood, double hoodState){
        this.hood = hood;
        this.hoodState = hoodState;

        addRequirements(hood);
    }

    @Override
    public void initialize(){
        System.out.println("hood state START: " + hoodState);
        hood.setState(hoodState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("hood state END INTERRUPTED: " + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return hood.didReachValue();
    }
}