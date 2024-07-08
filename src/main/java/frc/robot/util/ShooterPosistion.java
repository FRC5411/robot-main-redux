package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterPosistions {
    Rotation2d posisiton;
    double velocity;

    public ShooterPosistions(Rotation2d posistion, double velocity){
        this.posisiton = posistion;
        this.velocity = velocity;
    }
}
