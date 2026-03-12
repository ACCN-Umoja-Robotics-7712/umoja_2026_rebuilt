package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegree,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveReversed,
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    CANBus CANivore = new CANBus("CANivore");
    private final Pigeon2 gyro = new Pigeon2(1, CANivore);
    public PIDController shootController;
    public PIDController xController;
    public PIDController yController;
    public ProfiledPIDController thetaController;
    public HolonomicDriveController holonomicDriveController;
    public TrajectoryConfig trajectoryConfig;
    public final Timer timer = new Timer();
    // turretToTargetAngle current robot, assume turret is 0;
    private double turretToTargetAngle = 0;
    private double turretToTargetHoodValue = 0;
    private double turretToTargetRPMValue = 0;

    private double headingOffset = 0;
    
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, new Rotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),  
            backLeft.getPosition(),
            backRight.getPosition()
        }, new Pose2d()
    );
    
    RobotConfig config;

    /**
     * Hood encoder position lookup table (distance → encoder rotations from zero).
     * Measured during robot characterisation on 2026-03-11.
     * Hood stays at max (1.2) for distances beyond 3.5 m — only RPM changes at range.
     */
    private static final InterpolatingDoubleTreeMap hoodLookupTable = new InterpolatingDoubleTreeMap();
    static {
        //  distance (m)  →  hood encoder position (rotations from zeroed home)
        hoodLookupTable.put(1.50, 0.000);
        hoodLookupTable.put(1.75, 0.000);
        hoodLookupTable.put(2.00, 0.430);
        hoodLookupTable.put(2.25, 0.598);
        hoodLookupTable.put(2.50, 0.725);
        hoodLookupTable.put(2.75, 0.875);
        hoodLookupTable.put(3.00, 1.000);
        hoodLookupTable.put(3.25, 1.150);
        hoodLookupTable.put(3.50, 1.200);
        // Beyond 3.5 m the hood saturates at its max position (1.2);
        // only flywheel RPM is used to reach longer distances.
        hoodLookupTable.put(5.50, 1.200);
    }

    /**
     * Flywheel RPM lookup table (distance → RPM magnitude, always positive here).
     * Measured during robot characterisation on 2026-03-11.
     * "Tuned RPM" values used for 1.5–3.5 m; manufacturer-converted values used beyond.
     * The returned RPM from updateRPMHoodValues is negated because the motor runs reversed.
     */
    private static final InterpolatingDoubleTreeMap rpmLookupTable = new InterpolatingDoubleTreeMap();
    static {
        //  distance (m)  →  flywheel RPM (positive magnitude)
        rpmLookupTable.put(1.50, 3100.0);
        rpmLookupTable.put(1.75, 3200.0);
        rpmLookupTable.put(2.00, 3300.0);
        rpmLookupTable.put(2.25, 3350.0);
        rpmLookupTable.put(2.50, 3400.0);
        rpmLookupTable.put(2.75, 3450.0);
        rpmLookupTable.put(3.00, 3550.0);
        rpmLookupTable.put(3.25, 3650.0);
        rpmLookupTable.put(3.50, 3750.0);
        // From 3.75 m onwards only converted (formula) values are available;
        // replace these once tuned values are confirmed on the field.
        rpmLookupTable.put(3.75, 3954.0);
        rpmLookupTable.put(4.00, 4041.0);
        rpmLookupTable.put(4.25, 4126.0);
        rpmLookupTable.put(4.50, 4209.0);
        rpmLookupTable.put(4.75, 4291.0);
        rpmLookupTable.put(5.00, 4372.0);
        rpmLookupTable.put(5.25, 4451.0);
        rpmLookupTable.put(5.50, 4529.0);
    }

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);

                setHeading(0);
                resetEncoders();
                // frontRight.driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                // frontRight.turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
            }catch (Exception e) {
            }
        }).start();
    
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        // try{
            config = Constants.robotConfig;
        // } catch (Exception e) {
        // // Handle exception as needed
        //     e.printStackTrace();
        //     System.out.println("ROBOT GAVE UP PLEASE FIX CONFIG");
        //     return;
        // }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setModuleStatesFromSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, 0), // Translation PID constants
                        new PIDConstants(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, 0.0) // Rotation PID constants
                ), // TODO: Auto PID
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        
        trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 3. Define PID controllers for tracking trajectory
        shootController = new PIDController(0.1, 0, 0);
        
        xController = new PIDController(DriveConstants.kPDrive, DriveConstants.kIDrive, 0);
        yController = new PIDController(DriveConstants.kPDrive, DriveConstants.kIDrive, 0);
        thetaController = new ProfiledPIDController(
                DriveConstants.kPTurning, DriveConstants.kITurning, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(0,360);
        
        holonomicDriveController = new HolonomicDriveController(xController, yController, thetaController);

    }
    // Assuming this is a method in your drive subsystem
   public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaController.calculate(pose.getRotation().getDegrees(), Math.toDegrees(sample.heading))
        );

        // Apply the generated speeds
        setModuleStatesFromSpeeds(speeds);
    }

    public void resetEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setHeading(double offset){
        RobotContainer.wantedAngle = -1;
        gyro.reset();
        gyro.setYaw(offset);
        System.out.println("Gyro Reset");
    }

    public double getHeading(){
        double yaw = gyro.getYaw().getValueAsDouble();

        // if red flip
        if (!DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue)) {
            yaw += 180;
        }

        //Changes the -180->0 to 180->360 (0 to 180 stays the same)
        double heading = Math.IEEEremainder(yaw, 360);
        // double heading = (-gyro.getYaw() + 360)%180;
        if(heading < 0){
            heading = 180 + (180+heading);
        }
        SmartDashboard.putNumber("HEADING", heading);
        return heading;
    }

    // This is the same as getHeading but does not apply the alliance flip.
    // Use this when you need the raw gyro reading rather than the field-relative heading.
    public double getGlobalHeading() {
        double yaw = gyro.getYaw().getValueAsDouble();

        //Changes the -180->0 to 180->360 (0 to 180 stays the same)
        double heading = Math.IEEEremainder(yaw, 360);
        if(heading < 0){
            heading = 180 + (180+heading);
        }
        // BUG FIX: was publishing to "HEADING" key, overwriting the field-relative heading.
        SmartDashboard.putNumber("RAW_HEADING", heading);
        return heading;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        System.out.println("ODOMETRY RESET");
        poseEstimator.resetPosition(Rotation2d.fromDegrees(getHeading()), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    /**
     * Returns the robot's velocity in field-relative coordinates (X = away from blue wall,
     * Y = left when standing at blue alliance).
     * Used by the shoot-while-moving virtual-target calculation.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getRotation2d());
    }

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> turretPublisher = NetworkTableInstance.getDefault().getStructTopic("TurretPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("TargetPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> limelightPublishers = NetworkTableInstance.getDefault().getStructArrayTopic("LimelightPoses", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> allPointsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("AllPosesArray", Pose2d.struct).publish();

    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


    public void publishRobotPositions() {
        ArrayList<Pose2d> allPoints = new ArrayList<>();
        // allPoints.add(new Pose2d(3.568, 7.602, new Rotation2d(0)));
        // allPoints.add(new Pose2d(3.568, 7.602, new Rotation2d(0)));
        allPoints.add(Constants.SHOOTING_POSES.BLUE_DEPOT_CENTER);

        allPoints.add(Constants.SHOOTING_POSES.BLUE_DEPOT_CORNER);

        allPoints.add(Constants.SHOOTING_POSES.BLUE_HALF_LEFT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_HALF_RIGHT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_TRENCH_LEFT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_TRENCH_RIGHT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_HUB_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_OUTPOST_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_TOWER_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_DEPOT_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_DEPOT_CORNER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_HALF_LEFT);

        allPoints.add(Constants.SHOOTING_POSES.RED_HALF_RIGHT);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_HUB_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_NEUTRAL_LEFT);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_NEUTRAL_RIGHT);
        // BUG FIX: RED_NEUTRAL_RIGHT was mistakenly added twice; replaced with RED_NEUTRAL_LEFT_PICKUP.
        allPoints.add(Constants.SHOOTING_POSES.RED_NEUTRAL_LEFT_PICKUP);
        
        allPointsPublisher.set(allPoints.toArray(new Pose2d[0]));
    }

    @Override
    public void periodic() {
        
        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        
        swerveStatePublisher.set(moduleStates);
        SmartDashboard.putNumber("FL", frontLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("FR", frontRight.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BL", backLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BR", backRight.getAbsoluteEncoderDegree());

        SmartDashboard.putNumber("GAME STATE", RobotContainer.gameState);

        poseEstimator.update(Rotation2d.fromDegrees(getHeading()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
       
        // double up = Constants.TurretConstants.upOffset;
        // double yaw =  RobotContainer.shooterTurretSubsystem.getAngle();
        // double yaw = 0;
        // double pitch = Constants.TurretConstants.pitchOffset;
        // double roll = Constants.TurretConstants.rollOffset;
        // double forward = Constants.TurretConstants.turretCenterToCameraCentreLength + Math.sin(Math.toRadians(yaw)) * TurretConstants.turretCenterFromRobotCenterForwardLength;
        // double side = Constants.TurretConstants.turretCenterToCameraCentreLength + -Math.cos(Math.toRadians(yaw)) * TurretConstants.turretCenterFromRobotCenterSideLength;
        // LimelightHelpers.setCameraPose_RobotSpace(LimelightConstants.turretName, forward, side, up, roll, pitch, yaw + 180);
        // Pose3d turretCameraPose3d = LimelightHelpers.getCameraPose3d_RobotSpace(LimelightConstants.turretName);
        // Pose2d turretCameraRobotPose = turretCameraPose3d.toPose2d();

        // LimelightHelpers.SetRobotOrientation(LimelightConstants.turretName, getHeading(), 0, 0, 0, 0, 0);
        // LimelightHelpers.PoseEstimate turretMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.turretName);

        // SmartDashboard.putNumber("TURRET ANGLE", yaw);
        // boolean rejectTurretUpdate = false;

        // if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360 || RobotContainer.shooterTurretSubsystem.getVelocity() > 360) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        // {
        //     rejectTurretUpdate = true;
        // }
        // double visionTrustValue = 0.7;
        // if (RobotContainer.gameState == GameConstants.Disabled) {
        //     visionTrustValue = 0;
        // }
        // if (turretMT2 != null) {
        //     if (turretMT2.tagCount == 0) {
        //         rejectTurretUpdate = true;
        //     } else if (turretMT2.tagCount == 1) {
        //         visionTrustValue += 2;
        //     }

        //     if (!rejectTurretUpdate)
        //     {
        //         poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustValue,visionTrustValue,9999999));
        //         poseEstimator.addVisionMeasurement(
        //             turretMT2.pose,
        //             turretMT2.timestampSeconds);
        //     }
        // }
        
        ArrayList<Pose2d> limelightPoses = new ArrayList<>();
        
        String limelightRight = LimelightConstants.LIMELIGHT_RIGHT;
        LimelightHelpers.SetRobotOrientation(limelightRight, getHeading(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode(limelightRight, 4);
        LimelightHelpers.setCameraPose_RobotSpace(limelightRight, LimelightConstants.limelight4Forward, LimelightConstants.limelight4Side, LimelightConstants.limelight4Height, 0, LimelightConstants.limelight4Angle, 270);
        LimelightHelpers.PoseEstimate rightMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightRight);

        boolean rejectRightUpdate = false;

        if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectRightUpdate = true;
        }

        double visionTrustRightValue = 0.7;
        if (rightMT2 != null) {
            if (rightMT2.tagCount == 0) {
                rejectRightUpdate = true;
            } else if (rightMT2.tagCount == 1) {
                visionTrustRightValue += 1;
            }
            if (RobotContainer.gameState == GameConstants.Disabled) {
                visionTrustRightValue = 0;
            }
            if (!rejectRightUpdate)
            {
                limelightPoses.add(rightMT2.pose);
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustRightValue,visionTrustRightValue,9999999));
                poseEstimator.addVisionMeasurement(
                    rightMT2.pose,
                    rightMT2.timestampSeconds);
            }
        }

        String limelightForward = LimelightConstants.LIMELIGHT_FORWARD;
        LimelightHelpers.SetRobotOrientation(limelightForward, getHeading(), 0, 0, 0, 0, 0);
        // FIX: SetIMUMode(4) tells MegaTag2 to use the heading supplied by SetRobotOrientation
        // (i.e. the Pigeon2) instead of the limelight's own internal IMU, which has no knowledge
        // of the robot's true heading.  Without this call MegaTag2 poses are systematically wrong.
        LimelightHelpers.SetIMUMode(limelightForward, 4);
        LimelightHelpers.setCameraPose_RobotSpace(limelightForward, LimelightConstants.limelight2Forward, LimelightConstants.limelight2Side, LimelightConstants.limelight2Height, 0, LimelightConstants.limelight2Angle, 0);
        LimelightHelpers.PoseEstimate forwardMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightForward);

        boolean rejectForwardUpdate = false;

        if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectForwardUpdate = true;
        }

        // Trust value intentionally lower than limelight-four: tested and confirmed that
        // the older LL2/LL3 hardware is less pose-accurate than LL4.
        double visionTrustForwardValue = 2.7;
        if (forwardMT2 != null) {
            if (forwardMT2.tagCount == 0) {
                rejectForwardUpdate = true;
            } else if (forwardMT2.tagCount == 1) {
                visionTrustForwardValue += 2;
            }
            if (RobotContainer.gameState == GameConstants.Disabled) {
                visionTrustForwardValue = 0;
            } else {
                // Reject estimates that are unreasonably far from current odometry (bad tag read).
                // Threshold widened from 1.0 m → 2.5 m to avoid the chicken-and-egg problem:
                // a stale odometry from a single-camera drop shouldn't permanently block the
                // other two cameras from contributing updates.
                boolean isTooFarFromOdometry = forwardMT2.pose.getTranslation().getDistance(getPose().getTranslation()) > 2.5;
                // reject if outside arena 
                boolean isOutsideField = forwardMT2.pose.getX() < 0 || forwardMT2.pose.getX() > 17 || forwardMT2.pose.getY() > 8 || forwardMT2.pose.getY() < 0;
                if (isTooFarFromOdometry || isOutsideField) {
                    rejectForwardUpdate = true;
                }
            }
            if (!rejectForwardUpdate)
            {
                limelightPoses.add(forwardMT2.pose);
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustForwardValue,visionTrustForwardValue,9999999));
                poseEstimator.addVisionMeasurement(
                    forwardMT2.pose,
                    forwardMT2.timestampSeconds);
            }
        }
        String limelightLeft = LimelightConstants.LIMELIGHT_LEFT;
        LimelightHelpers.SetRobotOrientation(limelightLeft, getHeading(), 0, 0, 0, 0, 0);
        // FIX: SetIMUMode(4) — same reason as limelightForward above.  Without this the left
        // camera's MegaTag2 ignores the Pigeon2 heading and uses its internal IMU, producing
        // systematically incorrect poses.
        LimelightHelpers.SetIMUMode(limelightLeft, 4);
        LimelightHelpers.setCameraPose_RobotSpace(limelightLeft, LimelightConstants.limelight3Forward, LimelightConstants.limelight3Side, LimelightConstants.limelight3Height, 0, LimelightConstants.limelight3Angle, 90);
        LimelightHelpers.PoseEstimate leftMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightLeft);

        boolean rejectLeftUpdate = false;

        if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectLeftUpdate = true;
        }

        // Trust value intentionally lower than limelight-four: tested and confirmed that
        // the older LL2/LL3 hardware is less pose-accurate than LL4.
        double visionTrustLeftValue = 3.7;
        if (leftMT2 != null) {
            if (leftMT2.tagCount == 0) {
                rejectLeftUpdate = true;
            } else if (leftMT2.tagCount == 1) {
                visionTrustLeftValue += 2;
            }
            if (RobotContainer.gameState == GameConstants.Disabled) {
                visionTrustLeftValue = 0;
            } else {
                // Reject estimates that are unreasonably far from current odometry (bad tag read).
                // Threshold widened from 1.0 m → 2.5 m — same reasoning as limelightForward.
                boolean isTooFarFromOdometry = leftMT2.pose.getTranslation().getDistance(getPose().getTranslation()) > 2.5;
                // reject if outside arena
                boolean isOutsideField = leftMT2.pose.getX() < 0 || leftMT2.pose.getX() > 17 || leftMT2.pose.getY() > 8 || leftMT2.pose.getY() < 0;
                if (isTooFarFromOdometry || isOutsideField) {
                    rejectLeftUpdate = true;
                }
            }
            if (!rejectLeftUpdate)
            {
                limelightPoses.add(leftMT2.pose);
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustLeftValue,visionTrustLeftValue,9999999));
                poseEstimator.addVisionMeasurement(
                    leftMT2.pose,
                    leftMT2.timestampSeconds);
            }
        }
        
        Pose2d currentPose = getPose();

        if (RobotContainer.gameState == GameConstants.Disabled) {
            LimelightHelpers.SetThrottle(limelightLeft, 100);
        } else {
            LimelightHelpers.SetThrottle(limelightLeft, 0);
        }
        Pose2d turretFieldPose = currentPose.plus(new Transform2d(TurretConstants.turretCenterFromRobotCenterForwardLength, TurretConstants.turretCenterFromRobotCenterSideLength, new Rotation2d(Units.degreesToRadians(RobotContainer.shooterTurretSubsystem.getAngleDegrees()))));
        turretPublisher.set(turretFieldPose);
        limelightPublishers.set(limelightPoses.toArray(new Pose2d[0]));
        posePublisher.set(currentPose);

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue);

        Pose2d target = Constants.SHOOTING_POSES.BLUE_HUB_POSE;

        if (isBlue) {
            // .5 meter past blue hub into neutral zone
            if (getPose().getX() >= Constants.SHOOTING_POSES.BLUE_HUB_POSE.getX() + 0.5) {
                // want to pass
                // Greater than hub Y, on blue depot side
                if (getPose().getY() >= Constants.SHOOTING_POSES.BLUE_HUB_POSE.getY()) {
                    target = Constants.SHOOTING_POSES.BLUE_PASS_DEPOT_POSE;
                } else {
                    target = Constants.SHOOTING_POSES.BLUE_PASS_OUTPOST_POSE;
                }
            } else {
                target = Constants.SHOOTING_POSES.BLUE_HUB_POSE;
            }
        } else {
            // .5 meter past red hub into neutral zone
            if (getPose().getX() <= Constants.SHOOTING_POSES.RED_HUB_POSE.getX() - 0.5) {
                // want to pass
                // Greater than hub Y, on outpost side
                if (getPose().getY() >= Constants.SHOOTING_POSES.RED_HUB_POSE.getY()) {
                    target = Constants.SHOOTING_POSES.RED_PASS_OUTPOST_POSE;
                } else {
                    target = Constants.SHOOTING_POSES.RED_PASS_DEPOT_POSE;
                }
            } else {
                target = Constants.SHOOTING_POSES.RED_HUB_POSE;
            }
        }
        
        double[] angleDistance = updateTurretAngleDistanceToTarget(target);
        this.turretToTargetAngle = angleDistance[0];
        SmartDashboard.putNumber("WANTED TARGET ANGLE", turretToTargetAngle);
        SmartDashboard.putNumber("WANTED TURRET ANGLE", getTurretToTargetAngle());
        double distanceToTarget = angleDistance[1];
        SmartDashboard.putNumber("WANTED TARGET DISTANCE", distanceToTarget);
        double[] rpmHoodValues = updateRPMHoodValues(distanceToTarget);
        this.turretToTargetRPMValue = rpmHoodValues[0];
        this.turretToTargetHoodValue = rpmHoodValues[1];
        SmartDashboard.putNumber("WANTED RPM FROM DISTANCE", turretToTargetRPMValue);
        SmartDashboard.putNumber("WANTED HOOD FROM DISTANCE", turretToTargetHoodValue);
        // corner ~5.5 to 5.7m, tower ~2.9 to 3.1m, trench ~3.8m
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setModuleStatesFromSpeeds(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public Pose2d offsetPoint(Pose2d pose, double sideOffset) {
        Transform2d transform = new Transform2d(0, sideOffset, new Rotation2d(0));
        return pose.transformBy(transform);
    }
    
    public Pose2d offsetPoint(Pose2d pose, double sideOffset, double forwardOffset) {
        Transform2d transform = new Transform2d(forwardOffset, sideOffset, new Rotation2d(0));
        return pose.transformBy(transform);
    }

    // rotation in degrees
    public Pose2d offsetPoint(Pose2d pose, double sideOffset, double forwardOffset, double rotationOffset) {
        Transform2d transform = new Transform2d(forwardOffset, sideOffset, new Rotation2d(Math.toRadians(rotationOffset)));
        return pose.transformBy(transform);
    }

    public void alignWithTag(Double targetX, Double ySpeed, Double turningSpeed) {
        double currentX = 0;
        double xError = targetX - currentX;

        // Pose2d pose = getPose();

        // ChassisSpeeds robotLOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        ChassisSpeeds speeds = new ChassisSpeeds(
            shootController.calculate(xError, 0),
            ySpeed,
            turningSpeed
        );
        SmartDashboard.putNumber("TARGET X ERROR", xError);

        setModuleStatesFromSpeeds(speeds);
    }
    
    /**
     * Calculates the turret angle and effective distance to a target on the field,
     * compensating for the robot's own velocity so the robot can shoot accurately
     * while moving ("shoot while moving" / "shoot from anywhere").
     *
     * <p>Principle: the note inherits the robot's velocity at the moment of release.
     * If the robot is moving, the note will travel along the vector sum of
     * (ejection velocity) + (robot velocity). To still hit a stationary target we
     * must aim at a <em>virtual target</em>:
     *
     * <pre>
     *   virtualTarget = realTarget − robotVelocity × flightTime
     * </pre>
     *
     * Two Newton–Raphson iterations are used for accuracy (the flight time itself
     * depends on the adjusted distance, so we refine it once).
     *
     * <p>A SmartDashboard boolean key <b>"Shoot While Moving"</b> can be toggled to
     * disable compensation during testing/tuning (defaults to enabled).
     *
     * @param targetPose Field-relative pose of the target (only translation is used).
     * @return double[] {turretAngleDegrees, effectiveDistanceMeters}
     */
    public double[] updateTurretAngleDistanceToTarget(Pose2d targetPose) {
        targetPosePublisher.set(targetPose);

        // --- Raw geometry -------------------------------------------------------
        Translation2d robotTranslation = getPose().getTranslation();
        Translation2d toTarget = targetPose.getTranslation().minus(robotTranslation);
        double rawDistance = toTarget.getNorm();

        // --- Shoot-while-moving (virtual target) --------------------------------
        // Read the enable flag from SmartDashboard so it can be toggled in the pits.
        boolean shootWhileMoving = SmartDashboard.getBoolean("Shoot While Moving", true);
        SmartDashboard.putBoolean("Shoot While Moving", shootWhileMoving);

        Translation2d virtualTarget = toTarget; // start with raw vector

        if (shootWhileMoving) {
            ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();
            double vx = fieldSpeeds.vxMetersPerSecond;
            double vy = fieldSpeeds.vyMetersPerSecond;
            double projectileSpeed = TurretConstants.kProjectileSpeedMPS;

            // Iteration 1 — rough flight time from raw distance
            double flightTime = rawDistance / projectileSpeed;
            virtualTarget = toTarget.minus(new Translation2d(vx * flightTime, vy * flightTime));

            // Iteration 2 — refine using adjusted distance (Newton-Raphson step)
            flightTime = virtualTarget.getNorm() / projectileSpeed;
            virtualTarget = toTarget.minus(new Translation2d(vx * flightTime, vy * flightTime));

            SmartDashboard.putNumber("SWM Robot Vx (m/s)", vx);
            SmartDashboard.putNumber("SWM Robot Vy (m/s)", vy);
            SmartDashboard.putNumber("SWM Flight Time (s)", flightTime);
            SmartDashboard.putNumber("SWM Angle Correction (deg)",
                Units.radiansToDegrees(Math.atan2(virtualTarget.getY(), virtualTarget.getX()))
                - Units.radiansToDegrees(Math.atan2(toTarget.getY(), toTarget.getX())));
        }

        double turretAngleToTarget = Units.radiansToDegrees(
            Math.atan2(virtualTarget.getY(), virtualTarget.getX()));
        double effectiveDistance = virtualTarget.getNorm();

        SmartDashboard.putNumber("SWM Raw Distance (m)", rawDistance);
        SmartDashboard.putNumber("SWM Effective Distance (m)", effectiveDistance);

        return new double[]{turretAngleToTarget, effectiveDistance};
    }

    public double getRobotToTargetAngle() {
        return turretToTargetAngle;
    }
    
    public double getTurretToTargetAngle() {
        return (((turretToTargetAngle - getHeading())) + 360) % 360;
    }

    public double getTurretToTargetRPMValue() {
        return turretToTargetRPMValue;
    }

    public double getTurretToTargetHoodValue() {
        return turretToTargetHoodValue;
    }

    public double[] updateRPMHoodValues(double distanceToTarget) {
        // Clamp distance to characterised range so the table never extrapolates
        // wildly outside tested data (robot won't attempt shots from <1.5 m or >5.5 m).
        double clampedDistance = Math.max(1.50, Math.min(5.50, distanceToTarget));

        double rpm      = rpmLookupTable.get(clampedDistance);  // positive magnitude
        double hoodValue = hoodLookupTable.get(clampedDistance);

        SmartDashboard.putNumber("Lookup RPM", rpm);
        SmartDashboard.putNumber("Lookup Hood", hoodValue);

        // Motor runs in reverse, so RPM is negated on the way out.
        return new double[]{-rpm, hoodValue};
    }
}
