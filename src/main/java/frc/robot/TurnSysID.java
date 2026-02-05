package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class TurnSysID {

	public static CommandJoystick left_js = new CommandJoystick(4);
	SysIdRoutine routine;

	MutVoltage voltage = Volts.mutable(0);
	MutDistance distance = Meters.mutable(0);
	MutLinearVelocity velocity = MetersPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		// Robot.motor.setVoltage(volts.magnitude());
	}

	public void log(SysIdRoutineLog log) {
		// log.motor("motor")
		// .voltage(voltage.mut_replace(Robot.motor.getAppliedOutput()*Robot.motor.getBusVoltage(), Volts))
		// .linearVelocity(velocity.mut_replace(Robot.motor.getEncoder().getVelocity(), MetersPerSecond))
		// .linearPosition(distance.mut_replace(Robot.motor.getEncoder().getPosition(), Meters));
	}

	public void init() {
		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3), Seconds.of(10));
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, new Subsystem() {
			;
		});
		routine = new SysIdRoutine(config, mechanism);

		left_js.button(1).whileTrue(routine.quasistatic(Direction.kForward));
		left_js.button(2).whileTrue(routine.quasistatic(Direction.kReverse));
		left_js.button(3).whileTrue(routine.dynamic(Direction.kForward));
		left_js.button(4).whileTrue(routine.dynamic(Direction.kReverse));
	}
}
