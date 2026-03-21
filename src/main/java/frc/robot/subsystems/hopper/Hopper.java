// package frc.robot.subsystems.hopper;

// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
// import static edu.wpi.first.units.Units.Feet;
// import static edu.wpi.first.units.Units.Pounds;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import yams.gearing.GearBox;
// import yams.gearing.MechanismGearing;
// import yams.mechanisms.SmartMechanism;
// import yams.mechanisms.config.ArmConfig;
// import yams.mechanisms.positional.Arm;
// import yams.motorcontrollers.SmartMotorController;
// import yams.motorcontrollers.SmartMotorControllerConfig;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
// import yams.motorcontrollers.local.SparkWrapper;
// import yams.motorcontrollers.remote.TalonFXWrapper;
// import yams.telemetry.SmartMotorControllerTelemetryConfig;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// public class Hopper extends SubsystemBase {
    
//  private TalonFX swingTalonPrimary = new TalonFX(22, new CANBus());
//     private TalonFX swingTalonSecondary = new TalonFX(23, new CANBus());

//  SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//   .withMechanismPosition()
//   .withMechanismVelocity()
//   .withRotorPosition()
//   .withRotorVelocity()  
//   .withOutputVoltage()
//   .withStatorCurrent()
//   .withTelemetryVerbosity(TelemetryVerbosity.HIGH);


// private SmartMotorControllerConfig secondaryConfig = new SmartMotorControllerConfig(this)
//   .withControlMode(ControlMode.CLOSED_LOOP)
//   // Feedback Constants (PID Constants)
//   .withClosedLoopController(20, 0, 10, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(360))
//   .withSimClosedLoopController(1, 0, 0, DegreesPerSecond.of(15), DegreesPerSecondPerSecond.of(15))
//   // Feedforward Constants
//   .withFeedforward(new ArmFeedforward(0, 0, 0))
//   .withSimFeedforward(new ArmFeedforward(0, 0, 0))
//   // Telemetry name and verbosity level
//   .withTelemetry("IntakeMotor", motorTelemetryConfig)
//   // Gearing from the motor rotor to final shaft.
//   // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
//   // You could also use .withGearing(12) which does the same thing.
//   //.withGearing(new MechanismGearing(GearBox.fromStages("29:50","14:60","28:60")))
//   .withGearing(new MechanismGearing(GearBox.fromReductionStages(36, 25)))
//   // Motor properties to prevent over currenting.
//   .withMotorInverted(false)
//   .withIdleMode(MotorMode.BRAKE)
//   .withClosedLoopRampRate(Seconds.of(0.25))
//   .withOpenLoopRampRate(Seconds.of(0.25))
//   .withSupplyCurrentLimit(Amps.of(40));

//   private SmartMotorController talonControllerSecondary = new TalonFXWrapper(swingTalonSecondary, DCMotor.getKrakenX60(1), secondaryConfig); 


//  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
//   .withControlMode(ControlMode.CLOSED_LOOP)
//   // Feedback Constants (PID Constants)
//   .withClosedLoopController(20, 0, 10, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(360))
//   .withSimClosedLoopController(1, 0, 0, DegreesPerSecond.of(15), DegreesPerSecondPerSecond.of(15))
//   // Feedforward Constants
//   .withFeedforward(new ArmFeedforward(0, 0, 0))
//   .withSimFeedforward(new ArmFeedforward(0, 0, 0))
//   // Telemetry name and verbosity level
//   .withTelemetry("IntakeMotor", motorTelemetryConfig)
//   // Gearing from the motor rotor to final shaft.
//   // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
//   // You could also use .withGearing(12) which does the same thing.
//   //.withGearing(new MechanismGearing(GearBox.fromStages("29:50","14:60","28:60")))
//   .withGearing(new MechanismGearing(GearBox.fromReductionStages(36, 25)))
//   // Motor properties to prevent over currenting.
//   .withMotorInverted(false)
//   .withIdleMode(MotorMode.BRAKE)
//   .withClosedLoopRampRate(Seconds.of(0.25))
//   .withOpenLoopRampRate(Seconds.of(0.25))
//   .withSupplyCurrentLimit(Amps.of(40))
//   .withLooselyCoupledFollowers(talonControllerSecondary);


//   private SmartMotorController talonControllerPrimary = new TalonFXWrapper(swingTalonPrimary, DCMotor.getKrakenX60(1), smcConfig); 
  
//   private ArmConfig armCfg = new ArmConfig(talonControllerPrimary)
//   // Starting position is where your arm starts
//   .withStartingPosition(Degrees.of(0))
//   // Length and mass of your arm for sim.
//   .withLength(Feet.of(1))
//   .withMass(Pounds.of(1));

//   // Arm Mechanism
//   private Arm arm = new Arm(armCfg);

//   /** Creates a new ExampleSubsystem. */
//   public Hopper() {}

//   /**
//    * Move the arm up and down.
//    * @param hopper [-1, 1] speed to set the arm too.
//    */
//   public Command set(double hopper) { return arm.set(hopper);}
//  /**
//    * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the setpoint.
//    * @param angle Angle to go to.
//    * @return A Command
//    */
//   public Command setAngleAndStop(Angle angle, Angle tolerance) { return arm.runTo(angle, tolerance);}

//   public Command setAngle(Angle angle) {return arm.run(angle);}

//   /**
//    * Run sysId on the {@link Arm}
//    */

//   public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}
//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     arm.updateTelemetry();
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//     arm.simIterate();
//   }
// }
