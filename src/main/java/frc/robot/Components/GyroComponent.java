package frc.robot.Components;

import java.util.concurrent.locks.ReentrantLock;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroComponent {
    interface GyroIO{
        public void reset();
        public void reset(Rotation2d angle);
        public Rotation2d getRotation();
    }

    public static class NavX2Gyro implements GyroIO{
        private static ReentrantLock mutex = new ReentrantLock();
        private AHRS gyro;

        public NavX2Gyro(){
            mutex.lock();
            try{
                gyro = new AHRS(NavXComType.kMXP_SPI);
                while(gyro.isCalibrating());
                gyro.reset();
            } finally {
                mutex.unlock();
            }
        }
        public void reset(){
            gyro.zeroYaw();
        }
        public void reset(Rotation2d angle){
            gyro.setAngleAdjustment(angle.getDegrees() - gyro.getYaw());
        }
        public Rotation2d getRotation(){
            return gyro.getRotation2d();
        }
    }
}