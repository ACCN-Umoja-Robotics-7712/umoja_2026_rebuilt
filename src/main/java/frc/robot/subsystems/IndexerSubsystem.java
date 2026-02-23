// This is the subsystem for the indexer and kicker

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeRollerStates;

public class IndexerSubsystem extends SubsystemBase {
    private SparkFlex indexerMotor;
    private SparkFlex kickerMotor;
    
    public IndexerSubsystem() {
        indexerMotor = new SparkFlex(IndexerConstants.indexerMotorID, MotorType.kBrushless);
        SparkBaseConfig indexerConfig = new SparkFlexConfig().smartCurrentLimit(80); // NEO_Vortex
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerMotor = new SparkFlex(IndexerConstants.kickerMotorID, MotorType.kBrushless);
        SparkBaseConfig kickerConfig = new SparkFlexConfig().smartCurrentLimit(80); // NEO_Vortex (Current Limit is 80) 
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIndexer(double speed) { // Can change the speed for each motor independently
        indexerMotor.set(speed*0.26); // 30% was to weak to get the fuel out, 60% was okay, but the motor started jittering and sill needed some power. Will try 75%
        kickerMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer velocity", indexerMotor.getEncoder().getVelocity());
    }

}
