package frc.robot.commands.ZeroCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbZeroCommand extends Command {
    
    ClimbSubsystem climbSubsystem;

    public ClimbZeroCommand(ClimbSubsystem climbSubsystem){
        this.climbSubsystem = climbSubsystem;
    }

    @Override
    public void initialize(){
        System.out.println("Intake zero'd");
    }

    @Override
    public void execute(){
        this.climbSubsystem.zeroClimbPosition();
    }

    @Override
    public void end(boolean isInterrupted){
    }
}
