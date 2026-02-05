// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // public static SparkFlex motor;
  public static TalonFX motor, motor2;
  public CommandJoystick left_js = new CommandJoystick(4);
  public CommandJoystick right_js = new CommandJoystick(3);
  public CommandJoystick ds = new CommandJoystick(2);
  
  public PhotonCamera camera;

  // PIDController pid = new PIDController(3.596, 0, 0);
  PIDController pid = new PIDController(6e-3, 0, 2.5e-4);
  // PIDController pid = new PIDController(0, 0, 0);

  public static double MOTOR_kV = 0.0019203 / 1.07;
  public static double MOTOR_kS = MOTOR_kV * 30;


  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  TurnSysID sysid = new TurnSysID();

  @Override
  public void robotInit() {
    // motor = new SparkFlex(25, MotorType.kBrushless);
    motor = new TalonFX(0);
    motor2 = new TalonFX(2);
    motor2.setControl(new Follower(0, true));
    camera = new PhotonCamera("Arducam_OV9281_USB_Camera (1)")
    left_js.button(1).onTrue(new InstantCommand(() -> {
      double temp = store_speed;
      store_speed = sp_speed;
      sp_speed = temp;
    }));
    left_js.button(4).onTrue(new InstantCommand(() -> {
      sp_speed -= 100;
    }));
    left_js.button(3).onTrue(new InstantCommand(() -> {
      sp_speed += 100;
    }));
    // sysid.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Logger.recordOutput("Shooter/Speed", motor.getEncoder().getVelocity());
    // Logger.recordOutput("Shooter/DutyCycleOutput", motor.getAppliedOutput());
    // Logger.recordOutput("Shooter/Current", motor.getOutputCurrent());
    // Logger.recordOutput("Shooter/SetpointSpeed", sp_speed);
    // Logger.recordOutput("Shooter/BusVoltage", motor.getBusVoltage());

    Logger.recordOutput("Shooter/Speed", motor.getVelocity().getValueAsDouble() * 60);
    Logger.recordOutput("Shooter/DutyCycleOutput", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Shooter/Current", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/SetpointSpeed", sp_speed);
    Logger.recordOutput("Shooter/BusVoltage", motor.getSupplyVoltage().getValueAsDouble());

    ArrayList<Double> dists = new ArrayList<>();
    List<PhotonPipelineResult> res = camera.getAllUnreadResults();
    for(PhotonPipelineResult r : res) {
      for(PhotonTrackedTarget target : r.getTargets()) {
        if(target.fiducialId == 25 || target.fiducialId == 26) {
          dists.add(target.getBestCameraToTarget().getTranslation().getNorm());
        }
      }
    }
    double dist = 0;
    for(double d : dists) dist += d;
    if(dists.size()>0) dist /= dists.size();
    System.out.println(dists);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  double sp_speed = 0;
  double store_speed = -3500;
  @Override
  public void teleopPeriodic() {
    // double cur_speed = motor.getEncoder().getVelocity();
    double cur_speed = motor.getVelocity().getValueAsDouble() * 60;
    System.out.println(sp_speed + " " + store_speed);

    // motor.set(Math.abs(cur_speed) < Math.abs(sp_speed) ? Math.signum(sp_speed) : 0);
    // System.out.println(Math.abs(cur_speed) < Math.abs(sp_speed) ? Math.signum(sp_speed) : 0);
    motor.setVoltage(Math.signum(sp_speed) * MOTOR_kS + sp_speed * MOTOR_kV + pid.calculate(cur_speed, sp_speed));
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
