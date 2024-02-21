package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public SwerveDriveKinematics swerveKinematics = Constants.Swerve.swerveKinematics;
    public Translation2d frontLeftModule;
    public Translation2d frontRightModule;
    public Translation2d backLeftModule;
    public Translation2d backRightModule;
    public SwerveModuleState[] swerveModuleStates;
    private final Field2d m_field = new Field2d();
    private DoubleArrayPublisher moduleStatePublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("/ModuleStates").publish();
    private StructPublisher<Pose2d> posePublisher;

    public Eyes eyes;
    public StructArrayPublisher<SwerveModuleState> swerveKinematicsPublisher;
    public StructPublisher<Pose2d> estimatedRobotPosePublisher;

    public SwerveDrivePoseEstimator m_poseEstimator;


    public Swerve() {

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        eyes = new Eyes();

        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.gyroOffset);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

         
        Timer.delay(1.0);
        resetModulesToAbsolute();
        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getGyroYaw(), getModulePositions());
      
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("/MyPose", Pose2d.struct).publish();
        swerveKinematicsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveModuleStates", SwerveModuleState.struct).publish();
        estimatedRobotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/EstimatedRobotPose", Pose2d.struct).publish();

        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        Constants.Swerve.maxSpeed, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
      
      
        m_poseEstimator =
          new SwerveDrivePoseEstimator(
             Constants.Swerve.swerveKinematics,
             gyro.getRotation2d(),
             getModulePositions(),
             new Pose2d(),
             VecBuilder.fill(0.1, 0.1, 0.1),
             VecBuilder.fill(1.5, 1.5, 1.5)
          );
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    } 
    
    
    public ChassisSpeeds getChassisSpeed() {

        return swerveKinematics.toChassisSpeeds(getModuleStates());

    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeed) {

        SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(chassisSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        moduleStatePublisher.set(new double[] {
            desiredStates[0].angle.getDegrees(), desiredStates[0].speedMetersPerSecond,
            desiredStates[1].angle.getDegrees(), desiredStates[1].speedMetersPerSecond,
            desiredStates[2].angle.getDegrees(), desiredStates[2].speedMetersPerSecond,
            desiredStates[3].angle.getDegrees(), desiredStates[3].speedMetersPerSecond,
        });

        setModuleStates(desiredStates);
        
    }
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw(){
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getTargetRotation() {

        Pose2d robotPose = m_poseEstimator.getEstimatedPosition();
        Pose3d targetPose = eyes.getTargetPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));

        if (robotX > targetX) {

            angle = angle + 180;

        }

        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber(" inverted angle", -angle);

        return -angle;
    }

    public double getDistanceFromTarget() {

        double distance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) - Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) - Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("ChassisSpeedX", getChassisSpeed().vxMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedY", getChassisSpeed().vyMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedOmega", getChassisSpeed().omegaRadiansPerSecond);

        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        if (LimelightHelpers.getTV("") == true) {
            m_poseEstimator.addVisionMeasurement(
                eyes.getRobotPose(), 
                Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("")/1000.0) - (LimelightHelpers.getLatency_Capture("")/1000.0)
            );
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Robot X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("gyro angle", getGyroYaw().getDegrees());

        SmartDashboard.putNumber("Pose estimator rotations", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Pose Estimator X", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Estimator Y", m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("target X", eyes.getTargetPose().getX());
        SmartDashboard.putNumber("target Y", eyes.getTargetPose().getY());

        posePublisher.set(getPose());
        swerveKinematicsPublisher.set(getModuleStates());
        estimatedRobotPosePublisher.set(m_poseEstimator.getEstimatedPosition());
    }
}