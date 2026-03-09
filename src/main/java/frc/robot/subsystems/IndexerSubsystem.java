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

public class IndexerSubsystem extends SubsystemBase {
    private SparkFlex indexerMotorOriginal;
    private SparkFlex beltMotor;
    
    public IndexerSubsystem() {
        indexerMotorOriginal = new SparkFlex(IndexerConstants.indexerMotorLeaderID, MotorType.kBrushless);
        SparkBaseConfig indexerOriginalConfig = new SparkFlexConfig().smartCurrentLimit(40); // NEO_Vortex
        indexerMotorOriginal.configure(indexerOriginalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        beltMotor = new SparkFlex(IndexerConstants.indexerMotorFollowerID, MotorType.kBrushless);
        SparkBaseConfig beltConfig = new SparkFlexConfig().smartCurrentLimit(40); // NEO_Vortex
        beltConfig.inverted(true);
        beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIndexerAtVoltage(double voltage) { // Can change the speed for each motor independently
        indexerMotorOriginal.setVoltage(voltage);
        beltMotor.setVoltage(voltage/4.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer velocity", indexerMotorOriginal.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer Belt velocity", beltMotor.getEncoder().getVelocity());
    }

}
