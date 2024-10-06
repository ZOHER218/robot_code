package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.core.CvType;
import org.opencv.core.Rect;

public class Robot extends TimedRobot {
  Thread m_visionThread;

  private CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private CANSparkMax m_rightMotor = new CANSparkMax(3, MotorType.kBrushed);
  private DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  XboxController m_controller = new XboxController(0);

  private AnalogInput m_analogInput = new AnalogInput(0); // Create AnalogInput for pin 0
  private double centerX = 0.0;
  private final double CAMERA_WIDTH = 160.0;  // Assumed camera width

  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    m_rightMotor.setInverted(false);
    m_leftMotor.setInverted(true);

    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    //UsbCamera camera2 = CameraServer.startAutomaticCapture(1);
    camera.setResolution(160, 120);
    //camera2.setResolution(160, 120);

    // Start the vision processing thread
    m_visionThread = new Thread(() -> {
      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Processed", 160, 120);
      Mat mat = new Mat();

      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(mat) == 0) {
          outputStream.notifyError(cvSink.getError());
          continue;
        }

        // Convert to HSV for color detection
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(mat, hsvMat, Imgproc.COLOR_BGR2HSV);

        // Define the HSV range for the color blue
        Scalar lowerBlue = new Scalar(100, 150, 70); // Adjust these values based on lighting and camera
        Scalar upperBlue = new Scalar(130, 255, 255);

        // Create a mask for the blue color
        Mat mask = new Mat();
        Core.inRange(hsvMat, lowerBlue, upperBlue, mask);

        // Find contours of the blue object
        Moments moments = Imgproc.moments(mask);
        if (moments.get_m00() > 0) {
          centerX = moments.get_m10() / moments.get_m00(); // Calculate the center of the object
        }

        // Draw a circle at the center of the blue object
        Imgproc.circle(mat, new Point(centerX, 120), 5, new Scalar(0, 255, 0), -1);

        outputStream.putFrame(mat);  // Display the processed frame
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  @Override
  public void teleopPeriodic() {
    double speed = m_controller.getLeftY();
    double turn = m_controller.getRightX();
    double boost = m_controller.getRightTriggerAxis();

    // Apply manual joystick control in teleop mode
    m_robotDrive.arcadeDrive((speed) * (boost + 0.5), (-turn) * (boost + 0.5));

    SmartDashboard.putNumber("input 0", m_analogInput.getValue());
    SmartDashboard.putNumber("Joystick Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Joystick X", m_controller.getRightX());
    SmartDashboard.putNumber("Boost", m_controller.getRightTriggerAxis());
    SmartDashboard.putNumber("Analog Input", m_analogInput.getAverageVoltage()); // Display analog input value
    SmartDashboard.putNumber("Center X", centerX);  // Display the X position of the blue object
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

    if (centerX > 0) {
      double error = (centerX - (CAMERA_WIDTH / 2)) / (CAMERA_WIDTH / 2); // Error: -1 (left) to 1 (right)
      turnAdjustment = -error * 0.5;  // Adjust this value to control how aggressively the robot turns
    }

    // Drive the robot autonomously towards the blue object
    m_robotDrive.arcadeDrive(-speed, turnAdjustment);

    SmartDashboard.putNumber("input 0", m_analogInput.getValue());
    SmartDashboard.putNumber("Analog Input", m_analogInput.getAverageVoltage()); // Display analog input value
    SmartDashboard.putNumber("Center X", centerX);  // Display the X position of the blue object
    SmartDashboard.putData("Robot Drive", m_robotDrive);
  }
}
