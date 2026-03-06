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
    private SparkFlex indexerMotorLeader;
    private SparkFlex indexerMotorFollower;
    
    public IndexerSubsystem() {
        indexerMotorLeader = new SparkFlex(IndexerConstants.indexerMotorLeaderID, MotorType.kBrushless);
        SparkBaseConfig indexerLeadConfig = new SparkFlexConfig().smartCurrentLimit(40); // NEO_Vortex
        indexerMotorLeader.configure(indexerLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        indexerMotorFollower = new SparkFlex(IndexerConstants.indexerMotorFollowerID, MotorType.kBrushless);
        SparkBaseConfig indexerFollowConfig = new SparkFlexConfig().smartCurrentLimit(40); // NEO_Vortex
        indexerFollowConfig.follow(IndexerConstants.indexerMotorLeaderID, true);
        indexerMotorFollower.configure(indexerFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIndexer(double speed) { // Can change the speed for each motor independently
        indexerMotorLeader.set(speed*0.30); // 30% was to weak to get the fuel out, 60% was okay, but the motor started jittering and sill needed some power. Will try 75% (UPDATE: 50% is the most optimal as it worked really well)
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer velocity", indexerMotorLeader.getEncoder().getVelocity());
    }

}
