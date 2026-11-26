package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;;

public class SwerveDriveSubsystem extends SubsystemBase{

    private final SwerveModule frontLeftModule = new SwerveModule(
        SwerveConstants.FL_SPEED_ID, SwerveConstants.FL_TURN_ID, SwerveConstants.FL_TURN_ID, false);
    
    private final SwerveModule frontRightModule = new SwerveModule(
    SwerveConstants.FR_SPEED_ID, SwerveConstants.FR_TURN_ID, SwerveConstants.FR_TURN_ID, false);

    private final SwerveModule backLeftModule = new SwerveModule(
    SwerveConstants.BR_SPEED_ID, SwerveConstants.BR_TURN_ID, SwerveConstants.BR_TURN_ID, true);

    private final SwerveModule backRightModule = new SwerveModule(
        SwerveConstants.BL_SPEED_ID, SwerveConstants.BL_TURN_ID, SwerveConstants.BL_TURN_ID, false);
    
    public final Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON2_ID);
    public final SwerveDriveOdometry odometry;
    
    public SwerveDriveSubsystem(){
        resetField();
        odometry = new SwerveDriveOdometry(
            SwerveConstants.KINEMATICS, getRotation2d(), getModulePositions());

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch(Exception e){
            config = null;
            e.printStackTrace();
            System.out.println("n foi");
        }
        
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0, 0),
                new PIDConstants(5.0, 0, 0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }
        
    public void drive(double xspeed, double yspeed, double rotation){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xspeed, yspeed, rotation, getRotation2d());
            
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        
        driveRobotRelative(speeds);
    }
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        
        SwerveModuleState[] targetStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(targetSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
        
        frontLeftModule.setDesireState(targetStates[0]);
        frontRightModule.setDesireState(targetStates[1]);
        backRightModule.setDesireState(targetStates[2]);
        backLeftModule.setDesireState(targetStates[3]);
        
    }
    
    public void resetField(){
        gyro.setYaw(0);
    }
    
    public void resetPose(Pose2d pose){
        System.out.println(pose);
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(), frontRightModule.getPosition(),
            backRightModule.getPosition(), backLeftModule.getPosition()
        };
    }
    
    public SwerveModuleState[] getModuleState(){
        return new SwerveModuleState[]{
            frontLeftModule.getState(), frontRightModule.getState(),
            backRightModule.getState(), backLeftModule.getState()
        };
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleState());
    }

    @Override
    public void periodic(){
        odometry.update(getRotation2d(), getModulePositions());
    }
    
    private class SwerveModule{

        private final SparkMax m_speed;
        private final SparkClosedLoopController speedController;
        private final RelativeEncoder speedEncoder;
        
        private final SparkMax m_turn;
        private final SparkClosedLoopController turnController;
        private final RelativeEncoder turnEncoder;
        
        private final CANcoder canCoder;
        
        public SwerveModule(byte speedId, byte turnId, byte canCoderId, boolean driveInverted){

            m_speed = new SparkMax(speedId, MotorType.kBrushless);
            m_turn = new SparkMax(turnId, MotorType.kBrushless);

            speedEncoder = m_speed.getEncoder();
            turnEncoder = m_turn.getEncoder();
            
            speedController = m_speed.getClosedLoopController();
            turnController = m_turn.getClosedLoopController();

            canCoder = new CANcoder(canCoderId);
            
            SparkMaxConfig c_speed = new SparkMaxConfig();
            c_speed.inverted(driveInverted)
                   .idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(40);
            c_speed.encoder.positionConversionFactor(SwerveConstants.SPEED_POSITION_CONVERSION)
                           .velocityConversionFactor(SwerveConstants.SPEED_VELOCITY_CONVERSION);
            c_speed.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                              .pid(0.1, 0, 0);
            c_speed.closedLoop.feedForward.kV(12.0 / SwerveConstants.MAX_SPEED_METERS_PER_SECOND );

            m_speed.configure(c_speed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            SparkMaxConfig c_turn = new SparkMaxConfig();
            c_turn.inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20); 
            c_turn.encoder.positionConversionFactor(SwerveConstants.TURN_POSITION_CONVERSION);
            c_turn.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                             .pid(0.5, 0, 0)
                             .positionWrappingEnabled(true)
                             .positionWrappingInputRange(-Math.PI, Math.PI); // tentar mudar para rotação depois.
                             
            m_turn.configure(c_turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void resetToAbsolute() {
            double absolutePositionRot = canCoder.getAbsolutePosition().waitForUpdate(0.1).getValueAsDouble();
            double absolutePositionRad = absolutePositionRot * 2.0 * Math.PI;
            turnEncoder.setPosition(absolutePositionRad);
        }

        public SwerveModuleState getState(){
            return new SwerveModuleState(speedEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
        }

        public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(speedEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
        }

        public void setDesireState(SwerveModuleState desiredState){
            Rotation2d currentRotation = new Rotation2d(turnEncoder.getPosition());

            desiredState.optimize(currentRotation);
            desiredState.cosineScale(currentRotation);

            speedController.setSetpoint(desiredState.speedMetersPerSecond * 0.5, ControlType.kVelocity);
            turnController.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);

        }
    }
}
