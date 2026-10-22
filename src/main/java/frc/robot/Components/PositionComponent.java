package frc.robot.Components;

import java.util.function.Supplier;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Components.GyroComponent.*;

/**
 * A singleton class that encapsulates the {@link SwerveDrivePoseEstimator}
 * to give positions from limelight and odometry. (Uses blue origin)
 * @author Darren Ringer
 */
public class PositionComponent{
    //-------------------------------------------Constants--------------------------------------------//
    private static final double limelightUncertianty = 5.0;

    //-------------------------------------------Variables--------------------------------------------//
    private static PositionComponent instance;
    private static SwerveDrivePoseEstimator poseEstimator;
    private static Supplier<SwerveModulePosition[]> swerveModulePositionSupplier; 
    private static Supplier<SwerveModuleState[]> swerveModuleStatesSupplier;
    private static SwerveDriveKinematics kinematics;
    private static GyroIO gyro;
    private static Pose2d lastCache;

    //------------------------------------------Core Methods------------------------------------------//

    private PositionComponent(SwerveDriveKinematics m_kinematics, Supplier<SwerveModulePosition[]> m_swerveModulePositionsSupplier, 
                              Supplier<SwerveModuleState[]> m_swerveModuleStatesSupplier, Pose2d initialPose){
        gyro = new NavX2Gyro();
        gyro.reset();

        lastCache = initialPose;
        kinematics = m_kinematics;
        swerveModulePositionSupplier = m_swerveModulePositionsSupplier;
        swerveModuleStatesSupplier = m_swerveModuleStatesSupplier;
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation(), m_swerveModulePositionsSupplier.get(), initialPose);
    }
    /**
     * Gets the instance of PositionComponent or throws an error if none exists yet
     * @return Current instance of PositionComponent
     */
    public static PositionComponent getInstance(){
        if(instance == null){
            throw new Error("Cannot get instance of PositionComponent before initialization :(");
        }
        return instance;
    }
    /**
     * Initializes the PositionComponent
     * @param kinematics Swerve kinematics
     * @param swerveModulePositionsSupplier A supplier of the current drivetrain swerve module positions
     * @param swerveModuleStatesSupplier A supplier of the current drivetrain swerve module states
     * @param initialPose Starting Pose
     * @return Instance of PositionComponent
     */
    public static PositionComponent initialize(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier, 
                                               Supplier<SwerveModuleState[]> swerveModuleStatesSupplier, Pose2d initialPose){
        instance = new PositionComponent(kinematics, swerveModulePositionsSupplier, swerveModuleStatesSupplier, initialPose);
        return instance;
    }
    /**
     * PositionComponent periodic loop (should be called in ComponentManager.periodic())
     */
    public static void periodic(){
        lastCache = poseEstimator.update(gyro.getRotation(), swerveModulePositionSupplier.get());
        // if(!DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red){
        //     lastCache = new Pose2d(lastCache.getTranslation(),lastCache.getRotation().plus(Rotation2d.k180deg));
        // }
    }
    //--------------------------------------------Getters---------------------------------------------//

    /**
     * Fetches the current robot pose
     * @return The current RobotPose
     */
    public static Pose2d getPose2d(){
        return lastCache;
    }

    /**
     * Gets rotation reading from gyro
     * @return Gyro's rotation
     */
    public static Rotation2d getGyroReading(){
        return gyro.getRotation();
    }

    /**
     * Gets the robot's current chassis speeds
     * @return A {@link ChassisSpeeds} object with the robot's current speeds.
     */
    public static ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] swerveModuleStates = swerveModuleStatesSupplier.get();
        return kinematics.toChassisSpeeds(
            swerveModuleStates[0],
            swerveModuleStates[1],
            swerveModuleStates[2],
            swerveModuleStates[3]
        );
    }

    //--------------------------------------------Setters---------------------------------------------//

    /**
     * Sets the pose
     * @param newPose The new pose
     */
    public static void resetPose(Pose2d newPose){
        poseEstimator.resetPose(newPose);
        // gyro.reset(newPose.getRotation());
        lastCache = newPose;
    }
    
    /**
     * Rezeros pose
     */
    public static void zeroPose(){
        poseEstimator.resetPose(Pose2d.kZero);
        gyro.reset();
        lastCache = Pose2d.kZero;
    }
}