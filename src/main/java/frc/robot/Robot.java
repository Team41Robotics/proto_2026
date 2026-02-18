// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.lang.Math.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;

	// public static SparkFlex motor;
	public static TalonFX motor, motor2;
	public static TalonFX hood;
	// public CommandJoystick left_js = new CommandJoystick(4);
	// public CommandJoystick right_js = new CommandJoystick(3);
	// public CommandJoystick ds = new CommandJoystick(2);
	public CommandXboxController controller = new CommandXboxController(1);

	public PhotonCamera camera;

	// PIDController pid = new PIDController(3.596, 0, 0);
	// PIDController pid = new PIDController(6e-3, 0, 2.5e-4);
	PIDController pid = new PIDController(0, 0, 0);
	PIDController hpid = new PIDController(0.5, 0.1, 0);

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
		hood = new TalonFX(1);
		hood.setPosition(0);
		motor2.setControl(new Follower(0, MotorAlignmentValue.Opposed));

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		motor.getConfigurator().apply(config);
		motor2.getConfigurator().apply(config);
		motor2.setControl(new Follower(0, MotorAlignmentValue.Opposed));

		camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
		// left_js.button(1).onTrue(new InstantCommand(() -> {
		//   double temp = store_speed;
		//   store_speed = sp_speed;
		//   sp_speed = temp;
		// }));
		// left_js.button(4).onTrue(new InstantCommand(() -> {
		//   sp_speed -= 100;
		// }));
		// left_js.button(3).onTrue(new InstantCommand(() -> {
		//   sp_speed += 100;
		// }));
		controller.a().onTrue(new InstantCommand(() -> {
			double temp = store_speed;
			store_speed = sp_speed;
			sp_speed = temp;
		}));
		controller.x().onTrue(new InstantCommand(() -> {
			sp_speed -= 100;
		}));
		controller.b().onTrue(new InstantCommand(() -> {
			sp_speed += 100;
		}));

		// controller.x().onTrue(new InstantCommand(()->{
		//   hood.setPosition(0);
		// }));
		controller.povDown().onTrue(new InstantCommand(() -> {
			sp_hood += 10 / 180. * PI;
		}));
		controller.povUp().onTrue(new InstantCommand(() -> {
			sp_hood -= 10 / 180. * PI;
		}));
		// controller.y().onTrue(new InstantCommand(() -> {
		//   double temp = store_hood;
		//   store_hood = sp_hood;
		//   sp_hood = temp;
		// }));
		SmartDashboard.putData("hoodPID", hpid);
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
		Logger.recordOutput("Hood/Angle", hood.getPosition().getValueAsDouble() * 2 * PI);
		Logger.recordOutput("Hood/AngleSetpoint", sp_hood);
		Logger.recordOutput("Hood/Voltage", hood.getMotorVoltage().getValueAsDouble());

		ArrayList<Double> dists = new ArrayList<>();
		List<PhotonPipelineResult> res = camera.getAllUnreadResults();
		for (PhotonPipelineResult r : res) {
			for (PhotonTrackedTarget target : r.getTargets()) {
				if (target.fiducialId == 21 || target.fiducialId == 24) {
					dists.add(target.getBestCameraToTarget().getTranslation().getNorm());
				}
			}
		}
		double dist = 0;
		for (double d : dists) dist += d;
		if (dists.size() > 0) dist /= dists.size();
		Logger.recordOutput("Distt", dist);
		// System.out.println(dist);
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
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
	double store_speed = -4900;
	double sp_hood = 0;
	double store_hood = -2 * PI;

	@Override
	public void teleopPeriodic() {
		// double cur_speed = motor.getEncoder().getVelocity();
		double cur_speed = motor.getVelocity().getValueAsDouble() * 60;
		// System.out.println(sp_speed + " " + store_speed);

		// motor.set(Math.abs(cur_speed) < Math.abs(sp_speed) ? Math.signum(sp_speed) : 0);
		// System.out.println(Math.abs(cur_speed) < Math.abs(sp_speed) ? Math.signum(sp_speed) : 0);
		motor.setVoltage(Math.signum(sp_speed) * MOTOR_kS + sp_speed * MOTOR_kV + pid.calculate(cur_speed, sp_speed));

		hood.setVoltage(MathUtil.clamp(hpid.calculate(hood.getPosition().getValueAsDouble() * 2 * PI, sp_hood), -1, 1));
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
