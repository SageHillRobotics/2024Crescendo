package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void setFlywheelSpeed(double speed){
        leftFlywheel.set(speed);

    }
    public void ampSpeed(){
        leftFlywheel.set(.40);
        rightFlywheel.set(.40);
    }

    public void intake(){
        leftFlywheel.set(-.55);
        rightFlywheel.set(-.55);
        indexMotor.setIdleMode(IdleMode.kCoast);
    }

    public void brake(){
        indexMotor.setIdleMode(IdleMode.kBrake);
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
    public double getCurrent(){
        return leftFlywheel.getOutputCurrent();
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("current", getCurrent());
    }
}
