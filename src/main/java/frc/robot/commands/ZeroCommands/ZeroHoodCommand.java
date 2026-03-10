package frc.robot.commands.ZeroCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class ZeroHoodCommand extends Command {
    
    ShooterHoodSubsystem hoodSubsystem;

    public ZeroHoodCommand(ShooterHoodSubsystem hoodSubsystem){
        this.hoodSubsystem = hoodSubsystem;
    }

    @Override
    public void initialize(){
        System.out.println("Starting zeroing hood");
    }

    @Override
    public void execute(){
        this.hoodSubsystem.zeroHood();
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Zeroing hood finished inturrupted: " + isInterrupted);
        this.hoodSubsystem.runHood(0);
    }

    @Override
    public boolean isFinished() {
        return this.hoodSubsystem.finishedZeroing();
    }

      @Override
      public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
      }
}
