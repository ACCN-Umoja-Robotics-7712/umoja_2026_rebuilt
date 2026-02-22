package frc.robot.commands.ZeroCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class IntakeArmZeroCommand extends Command {
    
    ClimbSubsystem climbSubsystem;

    public IntakeArmZeroCommand(ClimbSubsystem climbSubsystem){
        this.climbSubsystem = climbSubsystem;
    }

    @Override
    public void initialize(){
        System.out.println("Intake zero'd");
    }

    @Override
    public void execute(){this.climbSubsystem.zeroClimbPosition();
    }

    @Override
    public void end(boolean isInterrupted){
        // System.out.println("Intake zero'd");
    }
}
