package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class ShooterHoodStateCommand extends Command{
    ShooterHoodSubsystem hood;
    double hoodState;

    public ShooterHoodStateCommand(ShooterHoodSubsystem hood, double hoodState){
        this.hood = hood;
        this.hoodState = hoodState;

        addRequirements(hood);
    }

    @Override
    public void initialize(){
        hood.setState(hoodState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("hood END");
    }

    @Override
    public boolean isFinished() {
        return hood.didReachState();
    }
}