package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  Thread m_visionThread;

  private CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private CANSparkMax m_rightMotor = new CANSparkMax(3, MotorType.kBrushed);
  private DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  XboxController m_controller = new XboxController(0);


  // Lifter
  private CANSparkMax joint_arm = new CANSparkMax(6, MotorType.kBrushed);
  private CANSparkMax slider = new CANSparkMax(5, MotorType.kBrushed);

  private AnalogInput m_analogInput = new AnalogInput(0); // Create AnalogInput for pin 0

  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    m_rightMotor.setInverted(false);
    m_leftMotor.setInverted(true);
    // slider.setIdleMode(IdleMode.kBrake);


  
  }

  @Override
  public void teleopPeriodic() {
    double speed = m_controller.getLeftY();
    double turn = m_controller.getRightX();
    double boost = m_controller.getRightTriggerAxis();

    // Apply manual joystick control in teleop mode
    m_robotDrive.arcadeDrive((speed) * (boost + 0.5), (-turn) * (boost + 0.5));


    // joint arm logic using lt and rt


    if (m_controller.getLeftBumper()) {
      slider.set(-0.25);
    } else if (m_controller.getRightBumper()) {
      slider.set(0.25);

    } else {
      slider.set(0);
      
    }

    
   
    
    // slider logic using arrows
    // slider.set(0.5);

    if (m_controller.getPOV() == 0) {
      joint_arm.set(0.5);
    } else if (m_controller.getPOV() == 180) {
      joint_arm.set(-0.5);
    } else {
      joint_arm.set(0);
    }

    
    
    // Dashboard
    SmartDashboard.putNumber("input 0", m_analogInput.getValue());
    SmartDashboard.putNumber("Joystick Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Joystick X", m_controller.getRightX());
    SmartDashboard.putNumber("Boost", m_controller.getRightTriggerAxis());
    SmartDashboard.putNumber("Analog Input", m_analogInput.getAverageVoltage()); // Display analog input value
    // SmartDashboard.putNumber("Center X", centerX);  // Display the X position of the blue object
    SmartDashboard.putData("Robot Drive", m_robotDrive);
  }

  @Override
  public void autonomousInit() {
    // Any initialization for autonomous can be added here.
  }

  @Override
  public void autonomousPeriodic() {
    // Modify turn based on the blue object position
    double turnAdjustment = 0.0;
    double speed = 0.5; // Constant speed forward

    // if (centerX > 0) {
    //   double error = (centerX - (CAMERA_WIDTH / 2)) / (CAMERA_WIDTH / 2); // Error: -1 (left) to 1 (right)
    //   turnAdjustment = -error * 0.5;  // Adjust this value to control how aggressively the robot turns
    // }

    // Drive the robot autonomously towards the blue object
    m_robotDrive.arcadeDrive(-speed, turnAdjustment);

    SmartDashboard.putNumber("input 0", m_analogInput.getValue());
    SmartDashboard.putNumber("Analog Input", m_analogInput.getAverageVoltage()); // Display analog input value
    // SmartDashboard.putNumber("Center X", centerX);  // Display the X position of the blue object
    SmartDashboard.putData("Robot Drive", m_robotDrive);
  }
}
