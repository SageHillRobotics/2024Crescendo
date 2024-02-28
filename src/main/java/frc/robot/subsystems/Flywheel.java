package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{
    public CANSparkMax leftFlywheel;
    public CANSparkMax rightFlywheel;
    public CANSparkMax indexMotor;
    public Flywheel(){
        rightFlywheel = new CANSparkMax(43, MotorType.kBrushless);
        rightFlywheel.setInverted(true);
        leftFlywheel = new CANSparkMax(44, MotorType.kBrushless);
        indexMotor = new CANSparkMax(42, MotorType.kBrushless);
    }
    public void spinFlywheel(){ 
        leftFlywheel.set(1);
        rightFlywheel.set(.75);
    }
    public void shoot(){
        indexMotor.set(1);
    }
    public void stopIndex(){
        indexMotor.stopMotor();
    }
    public void stopFlywheel(){
        rightFlywheel.stopMotor();
        leftFlywheel.stopMotor();
    }
    
}
