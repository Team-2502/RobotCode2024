package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkFlex;
import static com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.wpilibj.RobotBase.isReal;

/// Wrapper to use rev classes in rust, for whatever reason they cannot be accesed from rust directly
public class Wrapper {
    public static MotorType kBrushless() {
        return MotorType.kBrushless;
    }
    public static MotorType kBrushed() {
        return MotorType.kBrushed;
    }

    public static IdleMode kBrake() { return IdleMode.kBrake; }
    public static IdleMode kCoast() { return IdleMode.kCoast; }

    public static ControlType kPosition() { return ControlType.kPosition; }

    public static CANSparkFlex createSparkFlex(int id) { return new CANSparkFlex(id, MotorType.kBrushless); }

    public static void sparkFollow(CANSparkMax leader, CANSparkMax follower, boolean invert) { follower.follow(leader, invert); }

    public static void startNetworkTables() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        if (isReal()) {
            inst.startServer("/home/lvuser/networktables.json");
        } else {
            inst.startServer();
        }
    }

    public static AHRS createAHRS() {
        return new AHRS();
    }

    public static double getAngle(AHRS navx) {
        return navx.getAngle();
    }

    public static ControlMode ctreVelocity() {
        return ControlMode.Velocity;
    }

    public static ControlMode ctrePosition() {
        return ControlMode.Position;
    }

    public static ControlMode ctrePercent() {
      return ControlMode.PercentOutput;
    }

    public static double ctre6GetVelocity(com.ctre.phoenix6.hardware.TalonFX motor) {
      return motor.getVelocity().getValue();
    }

    /*public static void setSpeed(TalonFX motor, double speed) {
      motor.set(ControlMode.PercentOutput, speed);
    }*/

    //public static TalonFXInvertType TalonFXCounterClockwise() { return TalonFXInvertType.CounterClockwise; }

    public static SendableChooser<Integer> autoChooser() {
        SendableChooser<Integer> chooser = new SendableChooser<>();
        chooser.addOption("crooked", 1);
        chooser.setDefaultOption("straing", 2);
        chooser.addOption("tk", 3);

        SmartDashboard.putData(chooser);
        return chooser;
    }
}
