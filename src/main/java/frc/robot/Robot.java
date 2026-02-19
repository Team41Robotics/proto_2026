package frc.robot;

import static java.lang.Math.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * Robot implementing the BallTracker NetworkTables protocol.
 *
 * <p>The robot coordinates with a camera-side Python program that tracks yellow
 * ball trajectories. Communication happens over NT4 under the /BallTracker/ table.
 *
 * <p>See protocol.md for the full specification.
 */
public class Robot extends LoggedRobot {

	// ── Hardware ────────────────────────────────────────────────────────────
	/** Flywheel primary motor (CAN 0). */
	private TalonFX flywheel;
	/** Flywheel follower motor (CAN 2, opposed). */
	private TalonFX flywheel2;
	/** Hood / pivot motor (CAN 1). */
	private TalonFX hood;
	/** Indexer motor to feed balls into the shooter (CAN 3). */
	private TalonFX indexer;

	// ── Controls ────────────────────────────────────────────────────────────
	private final CommandXboxController controller = new CommandXboxController(1);

	// ── PID ─────────────────────────────────────────────────────────────────
	private final PIDController flywheelPid = new PIDController(0, 0, 0);
	private final PIDController hoodPid = new PIDController(0.5, 0.1, 0);

	// ── Feedforward constants ───────────────────────────────────────────────
	public static final double MOTOR_kV = 0.0019203 / 1.07;
	public static final double MOTOR_kS = MOTOR_kV * 30;
        
	// ── Tuning constants ────────────────────────────────────────────────────
	/** Voltage applied to indexer during ARMED (slow feed). */
	private static final double INDEXER_SLOW_VOLTAGE = 2.0;
	/** RPM tolerance — flywheel is "at speed" when error is within this. */
	private static final double FLYWHEEL_READY_THRESHOLD_RPM = 50.0;
	/** Hood angle tolerance (radians). */
	private static final double HOOD_READY_THRESHOLD_RAD = 2.0 * PI / 180.0;

	// ── Operator setpoints ──────────────────────────────────────────────────
	private double targetSpeedRpm = 0;
	private double storedSpeedRpm = -4900;
	private double targetHoodRad = 0;

	// ── Shot bookkeeping ────────────────────────────────────────────────────
	private int shotId = 0;
	private boolean shootReadySent = false;

	// ── NT: Robot → Camera publishers ───────────────────────────────────────
	private BooleanPublisher shootReadyPub;
	private IntegerPublisher shotIdPub;
	private DoublePublisher shotSpeedRpmPub;
	private DoublePublisher shotAngleDegPub;

	// ── NT: Camera → Robot subscribers ──────────────────────────────────────
	private StringSubscriber stateSub;
	private BooleanSubscriber okToShootSub;
	private BooleanSubscriber ballDetectedSub;
	private IntegerSubscriber shotCountSub;
	private DoubleSubscriber lastLandXSub;
	private DoubleSubscriber lastLandYSub;
	private DoubleSubscriber fpsSub;

	// ── Misc ────────────────────────────────────────────────────────────────
	private Command autonomousCommand;
	private final RobotContainer robotContainer;

	// =====================================================================
	// Constructor
	// =====================================================================
	public Robot() {
		// AdvantageKit logging
		Logger.recordMetadata("ProjectName", "BallTrackerProto");
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();

		robotContainer = new RobotContainer();
	}

	// =====================================================================
	// robotInit — hardware + NT setup
	// =====================================================================
	@Override
	public void robotInit() {
		// ── Motors ───────────────────────────────────────────────────────────
		flywheel = new TalonFX(0);
		flywheel2 = new TalonFX(2);
		hood = new TalonFX(1);
		indexer = new TalonFX(3);

		hood.setPosition(0);

		// Flywheel2 follows flywheel in opposite direction
		flywheel2.setControl(new Follower(0, MotorAlignmentValue.Opposed));

		TalonFXConfiguration coastConfig = new TalonFXConfiguration();
		coastConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		flywheel.getConfigurator().apply(coastConfig);
		flywheel2.getConfigurator().apply(coastConfig);
		flywheel2.setControl(new Follower(0, MotorAlignmentValue.Opposed));

		TalonFXConfiguration brakeConfig = new TalonFXConfiguration();
		brakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		indexer.getConfigurator().apply(brakeConfig);

		// ── NetworkTables — /BallTracker/ ───────────────────────────────────
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		NetworkTable table = nt.getTable("BallTracker");

		// Robot → Camera
		shootReadyPub = table.getBooleanTopic("robot/shoot_ready").publish();
		shotIdPub = table.getIntegerTopic("robot/shot_id").publish();
		shotSpeedRpmPub = table.getDoubleTopic("robot/shot_speed_rpm").publish();
		shotAngleDegPub = table.getDoubleTopic("robot/shot_angle_deg").publish();

		// Camera → Robot
		stateSub = table.getStringTopic("camera/state").subscribe("CALIBRATING");
		okToShootSub = table.getBooleanTopic("camera/ok_to_shoot").subscribe(false);
		ballDetectedSub = table.getBooleanTopic("camera/ball_detected").subscribe(false);
		shotCountSub = table.getIntegerTopic("camera/shot_count").subscribe(0);
		lastLandXSub = table.getDoubleTopic("camera/last_land_x_cm").subscribe(0.0);
		lastLandYSub = table.getDoubleTopic("camera/last_land_y_cm").subscribe(0.0);
		fpsSub = table.getDoubleTopic("camera/fps").subscribe(0.0);

		// Initialise published values
		shootReadyPub.set(false);
		shotIdPub.set(0);
		shotSpeedRpmPub.set(0.0);
		shotAngleDegPub.set(0.0);

		// ── Controller bindings ─────────────────────────────────────────────
		// A — swap between active speed and stored speed
		controller.a().onTrue(new InstantCommand(() -> {
			double tmp = storedSpeedRpm;
			storedSpeedRpm = targetSpeedRpm;
			targetSpeedRpm = tmp;
		}));
		// X — decrease target speed by 100 RPM
		controller.x().onTrue(new InstantCommand(() -> targetSpeedRpm -= 100));
		// B — increase target speed by 100 RPM
		controller.b().onTrue(new InstantCommand(() -> targetSpeedRpm += 100));
		// POV Down — increase hood angle by 10°
		controller.povDown().onTrue(new InstantCommand(() -> targetHoodRad += 10.0 / 180.0 * PI));
		// POV Up — decrease hood angle by 10°
		controller.povUp().onTrue(new InstantCommand(() -> targetHoodRad -= 10.0 / 180.0 * PI));

		SmartDashboard.putData("hoodPID", hoodPid);
	}

	// =====================================================================
	// robotPeriodic — logging & NT reads
	// =====================================================================
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		// ── Read camera values ──────────────────────────────────────────────
		String cameraState = stateSub.get();
		boolean okToShoot = okToShootSub.get();
		boolean ballDetected = ballDetectedSub.get();
		long shotCount = shotCountSub.get();
		double landX = lastLandXSub.get();
		double landY = lastLandYSub.get();
		double cameraFps = fpsSub.get();

		// ── AdvantageKit logging ────────────────────────────────────────────
		double flywheelRpm = flywheel.getVelocity().getValueAsDouble() * 60.0;
		Logger.recordOutput("Shooter/Speed", flywheelRpm);
		Logger.recordOutput("Shooter/Voltage", flywheel.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Shooter/Current", flywheel.getStatorCurrent().getValueAsDouble());
		Logger.recordOutput("Shooter/SetpointSpeed", targetSpeedRpm);
		Logger.recordOutput("Shooter/BusVoltage", flywheel.getSupplyVoltage().getValueAsDouble());
		Logger.recordOutput("Hood/Angle", hood.getPosition().getValueAsDouble() * 2.0 * PI);
		Logger.recordOutput("Hood/AngleSetpoint", targetHoodRad);
		Logger.recordOutput("Hood/Voltage", hood.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Indexer/Voltage", indexer.getMotorVoltage().getValueAsDouble());

		Logger.recordOutput("BallTracker/CameraState", cameraState);
		Logger.recordOutput("BallTracker/OkToShoot", okToShoot);
		Logger.recordOutput("BallTracker/BallDetected", ballDetected);
		Logger.recordOutput("BallTracker/ShotCount", shotCount);
		Logger.recordOutput("BallTracker/LandX_cm", landX);
		Logger.recordOutput("BallTracker/LandY_cm", landY);
		Logger.recordOutput("BallTracker/CameraFPS", cameraFps);
		Logger.recordOutput("BallTracker/ShotId", shotId);
		Logger.recordOutput("BallTracker/ShootReadySent", shootReadySent);
	}

	// =====================================================================
	// Teleop — BallTracker state machine
	// =====================================================================
	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		// Reset protocol handshake state on teleop entry
		shootReadySent = false;
		shootReadyPub.set(false);
	}

	@Override
	public void teleopPeriodic() {
		String cameraState = stateSub.get();
		boolean okToShoot = okToShootSub.get();

		// Flywheel and hood run continuously — always spin up and position
		runFlywheel(targetSpeedRpm);
		runHood(targetHoodRad);

		// Only the indexer is gated by protocol state
		switch (cameraState) {
			case "CALIBRATING":
				handleCalibrating();
				break;
			case "WAITING":
				handleWaiting(okToShoot);
				break;
			case "ARMED":
				handleArmed();
				break;
			case "TRACKING":
				handleTracking();
				break;
			case "COOLDOWN":
				handleCooldown();
				break;
			case "DONE":
				handleDone();
				break;
			default:
				// Unknown state — stop indexer only
				stopIndexer();
				break;
		}
	}

	@Override
	public void teleopExit() {
		stopIndexer();
		shootReadyPub.set(false);
	}

	// =====================================================================
	// State handlers
	// =====================================================================

	/**
	 * CALIBRATING — camera is detecting AprilTags. Indexer off; flywheel + hood keep running.
	 */
	private void handleCalibrating() {
		stopIndexer();
		shootReadySent = false;
	}

	/**
	 * WAITING — camera is ready (ok_to_shoot = true). Flywheel and hood are already
	 * running continuously. Once both are at setpoint, set shoot_ready = true.
	 */
	private void handleWaiting(boolean okToShoot) {
		// Indexer stays off during WAITING
		stopIndexer();

		// Reset shoot_ready flag each time we are in WAITING (after DONE cleared it)
		if (shootReadySent) {
			shootReadySent = false;
			shootReadyPub.set(false);
		}

		if (!okToShoot) {
			// Camera not ready yet — don't signal
			return;
		}

		// Check if flywheel and hood are at setpoint
		double flywheelRpm = flywheel.getVelocity().getValueAsDouble() * 60.0;
		double hoodAngleRad = hood.getPosition().getValueAsDouble() * 2.0 * PI;
		boolean flywheelReady = abs(flywheelRpm - targetSpeedRpm) < FLYWHEEL_READY_THRESHOLD_RPM;
		boolean hoodReady = abs(hoodAngleRad - targetHoodRad) < HOOD_READY_THRESHOLD_RAD;

		if (flywheelReady && hoodReady && abs(targetSpeedRpm) > 0) {
			// Publish shot metadata then signal shoot_ready
			shotId++;
			shotIdPub.set(shotId);
			shotSpeedRpmPub.set(targetSpeedRpm);
			shotAngleDegPub.set(targetHoodRad / PI * 180.0);
			shootReadyPub.set(true);
			shootReadySent = true;
		}
	}

	/**
	 * ARMED — camera is watching for the ball. Flywheel + hood already running.
	 * Slowly feed the indexer to push a ball into the shooter.
	 */
	private void handleArmed() {
		// Slowly feed ball — flywheel and hood are already being driven above
		indexer.setVoltage(INDEXER_SLOW_VOLTAGE);
	}

	/**
	 * TRACKING — ball is in flight. Stop indexer only; flywheel + hood keep running.
	 */
	private void handleTracking() {
		stopIndexer();
	}

	/**
	 * COOLDOWN — ball may be bouncing. Stop indexer only; flywheel + hood keep running.
	 */
	private void handleCooldown() {
		stopIndexer();
	}

	/**
	 * DONE — shot recorded, momentary state. Clear shoot_ready; stop indexer.
	 */
	private void handleDone() {
		stopIndexer();
		// Clear shoot_ready so camera sees the falling edge
		shootReadySent = false;
		shootReadyPub.set(false);
	}

	// =====================================================================
	// Motor helpers
	// =====================================================================

	/** Run the flywheel at the given RPM using feedforward + PID. */
	private void runFlywheel(double setpointRpm) {
		double currentRpm = flywheel.getVelocity().getValueAsDouble() * 60.0;
		double voltage = signum(setpointRpm) * MOTOR_kS
				+ setpointRpm * MOTOR_kV
				+ flywheelPid.calculate(currentRpm, setpointRpm);
		flywheel.setVoltage(voltage);
	}

	/** Run the hood to the given angle (radians) using PID. */
	private void runHood(double setpointRad) {
		double currentRad = hood.getPosition().getValueAsDouble() * 2.0 * PI;
		double voltage = MathUtil.clamp(hoodPid.calculate(currentRad, setpointRad), -1, 1);
		hood.setVoltage(voltage);
	}

	/** Stop the indexer only. Flywheel and hood are never stopped by the state machine. */
	private void stopIndexer() {
		indexer.setVoltage(0);
	}

	// =====================================================================
	// Autonomous
	// =====================================================================
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();
		if (autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	// =====================================================================
	// Disabled
	// =====================================================================
	@Override
	public void disabledInit() {
		stopIndexer();
		shootReadyPub.set(false);
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	// =====================================================================
	// Test
	// =====================================================================
	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
