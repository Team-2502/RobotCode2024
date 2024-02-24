package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkFlex;
import static com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
        chooser.addOption("pass on right", 1);
        chooser.setDefaultOption("top", 4);
        chooser.addOption("eco", 3);
        chooser.addOption("test", 2);

        SmartDashboard.putData(chooser);
        return chooser;
    }

    private static final Field2d m_field = new Field2d();

    public static void setPosition(double x, double y, double theta) {
        m_field.setRobotPose(x, y, Rotation2d.fromRadians(theta));

        SmartDashboard.putData("field", m_field);
    }
}
