package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class ManualIntakeArmCommand extends Command{
    IntakeArmSubsystem intakeArm;
    Supplier<Double> intakeArmSpeed;

    public ManualIntakeArmCommand(IntakeArmSubsystem intakeArm, Supplier<Double> intakeArmSpeed){
        this.intakeArm = intakeArm;
        this.intakeArmSpeed = intakeArmSpeed;

        addRequirements(intakeArm);
    }

    @Override
    public void initialize(){
        System.out.println("Manual Intake Arm Initialized");
    }

    @Override
    public void execute(){
        intakeArm.runIntakeArm(intakeArmSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Intake Arm end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    