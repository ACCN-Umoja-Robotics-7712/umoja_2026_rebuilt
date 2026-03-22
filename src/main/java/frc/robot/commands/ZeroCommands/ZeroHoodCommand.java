package frc.robot.commands.ZeroCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class ZeroHoodCommand extends Command {
    
    ShooterHoodSubsystem hoodSubsystem;

    public ZeroHoodCommand(ShooterHoodSubsystem hoodSubsystem){
        this.hoodSubsystem = hoodSubsystem;
        // BUG FIX: addRequirements was missing — without it the scheduler doesn't know
        // this command owns the hood subsystem, so other hood commands can run
        // simultaneously during zeroing and fight for the same motor.
        addRequirements(hoodSubsystem);
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
