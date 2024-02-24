package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Joint.JointState;
import frc.robot.subsystems.Toaster.ToasterState;
import frc.robot.subsystems.Wrist.WristState;

public class SuperStructure extends SubsystemBase {

  public static final Wrist m_wrist = new Wrist();
  public static final Elevator m_elevator = new Elevator();
  public static final Indexer m_indexer = new Indexer();
  public static final Toaster m_toaster = new Toaster();
  public static final Joint m_joint = new Joint();

  public SuperStructure() {}

  private enum SuperStructureState{
    // OFF,
    ZERO,
    GROUND,
    AMP,
    SOURCE,
    SHOOT,
    TRAP
  }

  
  
  private SuperStructureState m_state = SuperStructureState.ZERO;

  
  public void zeroAll(){
    m_wrist.setState(WristState.ZERO);
    m_elevator.setState(ElevatorState.ZERO);
    m_joint.setState(JointState.ZERO);
    m_toaster.setState(ToasterState.OFF);
    m_indexer.setState(IndexerState.OFF);
  }

  public void shoot(double RPM, Rotation2d setpoint){
    m_joint.setSetpoint(setpoint);
    m_toaster.setRPM(RPM);
  }

  
  public void wristGoToSetpoint(Rotation2d setpoint){
    m_wrist.setSetpoint(setpoint);
  }
  public void elevatorGoToSetpoint(Rotation2d setpoint){
    m_elevator.setSetpoint(setpoint);
  }
  public void jointrGoToSetpoint(Rotation2d setpoint){
    m_joint.setSetpoint(setpoint);
  }

  //   public void goToSetpoint(Rotation2d wristSetpoint, Rotation2d elevatorSetpoint, Rotation2d jointSetpoint){
  //   m_wrist.setSetpoint(wristSetpoint);
  //   m_elevator.setSetpoint(elevatorSetpoint);
  //   m_joint.setSetpoint(jointSetpoint);
  // }

  public void intake(){
    m_toaster.setState(ToasterState.INTAKE);
    m_indexer.setState(IndexerState.ON);
  }

  public void setState(SuperStructureState newState) {
    m_state = newState;
    switch (m_state) {
      case ZERO:
        zeroAll();
        break;
      case GROUND:
        //wrist comes out
        //elevator comes out a bit
        //joint goes down all the way past the zero point
        // goToSetpoint(null, null, null);
        m_wrist.goToSetpoint();
        elevatorGoToSetpoint(null);
        jointrGoToSetpoint(null);
        intake();
        break;
      case AMP:
        //add amp assist subsystem
        m_wrist.setState(WristState.ZERO);
        //elevator out
        //joint up
        
        // goToSetpoint(null, null, null);
        shoot(0, null);

        break;
      case SOURCE:
        m_wrist.setState(WristState.ZERO);
        // goToSetpoint(null, null, null);
        
        break;
      case SHOOT:
        //photonvisionnnn
        shoot(100, Rotation2d.fromRotations(0));
        break;
      case TRAP:
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
