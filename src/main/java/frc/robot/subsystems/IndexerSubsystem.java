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
import frc.robot.Constants.TurretConstants;

public class IndexerSubsystem extends SubsystemBase {
    private SparkFlex indexerMotorOriginal;
    private SparkFlex indexerMotorGearedDown;
    
    public IndexerSubsystem() {
        indexerMotorOriginal = new SparkFlex(IndexerConstants.indexerMotorLeaderID, MotorType.kBrushless);
        SparkBaseConfig indexerOriginalConfig = new SparkFlexConfig().smartCurrentLimit(40); // NEO_Vortex
        indexerMotorOriginal.configure(indexerOriginalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        indexerMotorGearedDown = new SparkFlex(IndexerConstants.indexerMotorFollowerID, MotorType.kBrushless);
        SparkBaseConfig indexerFollowConfig = new SparkFlexConfig().smartCurrentLimit(40); // NEO_Vortex
        indexerMotorGearedDown.configure(indexerFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIndexerAtVoltage(double voltage) { // Can change the speed for each motor independently
        indexerMotorOriginal.setVoltage(voltage/9.0);
        indexerMotorGearedDown.setVoltage(voltage); // 30% was to weak to get the fuel out, 60% was okay, but the motor started jittering and sill needed some power. Will try 75% (UPDATE: 50% is the most optimal as it worked really well)
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer velocity", indexerMotorOriginal.getEncoder().getVelocity());
    }

}
