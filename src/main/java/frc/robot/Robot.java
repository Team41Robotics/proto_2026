package frc.robot;

import static java.lang.Math.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
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
	private SparkFlex flywheel;
	/** Flywheel follower motor (CAN 2, opposed). */
	private SparkFlex flywheel2;
	/** Indexer motor to feed balls into the shooter (CAN 3). */
	private SparkFlex indexer;

	// ── Controls ────────────────────────────────────────────────────────────
	private final CommandXboxController controller = new CommandXboxController(1);

	// ── PID ─────────────────────────────────────────────────────────────────
	private final PIDController flywheelPid = new PIDController(0, 0, 0);

	// ── Feedforward constants ───────────────────────────────────────────────
	public static final double MOTOR_kV = 0.0019203;
	public static final double MOTOR_kS = MOTOR_kV * 30;

	// ── Tuning constants ────────────────────────────────────────────────────
	/** Voltage applied to indexer during ARMED (slow feed). */
	private static final double INDEXER_SLOW_VOLTAGE = 2.0;
	/** RPM tolerance — flywheel is "at speed" when error is within this. */
	private static final double FLYWHEEL_READY_THRESHOLD_RPM = 50.0;

	// ── Operator setpoints ──────────────────────────────────────────────────
	private double targetSpeedRpm = 0;
	private double storedSpeedRpm = -4900;

	// ── Shot bookkeeping ────────────────────────────────────────────────────
	private int shotId = 0;
	private boolean shootReadySent = false;

	// ── NT: Robot → Camera publishers ───────────────────────────────────────
	private BooleanPublisher shootReadyPub;
	private IntegerPublisher shotIdPub;
	private DoublePublisher shotSpeedRpmPub;

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
		flywheel = new SparkFlex(41, MotorType.kBrushless);
		flywheel2 = new SparkFlex(26, MotorType.kBrushless);
		indexer = new SparkFlex(25, MotorType.kBrushless);

		SparkFlexConfig flywheelConfig = new SparkFlexConfig();
		flywheelConfig.idleMode(IdleMode.kCoast);
		flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// flywheel2 follows flywheel in opposite direction
		SparkFlexConfig flywheel2Config = new SparkFlexConfig();
		flywheel2Config.idleMode(IdleMode.kCoast).follow(flywheel, true);
		flywheel2.configure(flywheel2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkFlexConfig indexerConfig = new SparkFlexConfig();
		indexerConfig.idleMode(IdleMode.kBrake);
		indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// ── NetworkTables — /BallTracker/ ───────────────────────────────────
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		NetworkTable table = nt.getTable("BallTracker");

		// Robot → Camera
		shootReadyPub = table.getBooleanTopic("robot/shoot_ready").publish();
		shotIdPub = table.getIntegerTopic("robot/shot_id").publish();
		shotSpeedRpmPub = table.getDoubleTopic("robot/shot_speed_rpm").publish();

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
		double flywheelRpm = flywheel.getEncoder().getVelocity();
		Logger.recordOutput("Shooter/Speed", flywheelRpm);
		Logger.recordOutput("Shooter/Voltage", flywheel.getAppliedOutput() * flywheel.getBusVoltage());
		Logger.recordOutput("Shooter/Current", flywheel.getOutputCurrent());
		Logger.recordOutput("Shooter/SetpointSpeed", targetSpeedRpm);
		Logger.recordOutput("Shooter/BusVoltage", flywheel.getBusVoltage());
		Logger.recordOutput("Indexer/Voltage", indexer.getAppliedOutput() * indexer.getBusVoltage());

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

		// Flywheel runs continuously — always spin up
		runFlywheel(targetSpeedRpm);

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
	 * CALIBRATING — camera is detecting AprilTags. Indexer off; flywheel keeps running.
	 */
	private void handleCalibrating() {
		stopIndexer();
		shootReadySent = false;
	}

	/**
	 * WAITING — camera is ready (ok_to_shoot = true). Flywheel is already
	 * running continuously. Once it is at setpoint, set shoot_ready = true.
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

		// Check if flywheel is at setpoint
		double flywheelRpm = flywheel.getEncoder().getVelocity();
		boolean flywheelReady = abs(flywheelRpm - targetSpeedRpm) < FLYWHEEL_READY_THRESHOLD_RPM;

		if (flywheelReady && abs(targetSpeedRpm) > 0) {
			// Publish shot metadata then signal shoot_ready
			shotId++;
			shotIdPub.set(shotId);
			shotSpeedRpmPub.set(targetSpeedRpm);
			shootReadyPub.set(true);
			shootReadySent = true;
		}
	}

	/**
	 * ARMED — camera is watching for the ball. Flywheel already running.
	 * Slowly feed the indexer to push a ball into the shooter.
	 */
	private void handleArmed() {
		// Slowly feed ball — flywheel is already being driven above
		indexer.setVoltage(INDEXER_SLOW_VOLTAGE);
	}

	/**
	 * TRACKING — ball is in flight. Stop indexer only; flywheel keeps running.
	 */
	private void handleTracking() {
		stopIndexer();
	}

	/**
	 * COOLDOWN — ball may be bouncing. Stop indexer only; flywheel keeps running.
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
		double currentRpm = flywheel.getEncoder().getVelocity();
		double voltage = signum(setpointRpm) * MOTOR_kS
				+ setpointRpm * MOTOR_kV
				+ flywheelPid.calculate(currentRpm, setpointRpm);
		flywheel.setVoltage(voltage);
	}

	/** Stop the indexer only. Flywheel is never stopped by the state machine. */
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
