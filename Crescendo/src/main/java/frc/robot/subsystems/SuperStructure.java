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

  public final Wrist m_wrist = new Wrist();
  public final Hood m_hood = new Hood();
  public final Elevator m_elevator = new Elevator();
  public final Indexer m_indexer = new Indexer();
  public final Toaster m_toaster = new Toaster();
  public final Joint m_joint = new Joint();

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

  public void shoot(double RPM){
    m_toaster.setRPM(RPM);
  }

  //Count combine setpoint methonds for simplification
  public void wristGoToSetpoint(Rotation2d setpoint){
    m_wrist.setSetpoint(setpoint);
  }

  public void elevatorGoToSetpoint(double setpoint){
    m_elevator.setSetpoint(setpoint);
  }
  public void jointGoToSetpoint(Rotation2d setpoint){
    m_joint.setSetpoint(setpoint);
  }
  public void hoodGoToSetpoint(Rotation2d setpoint){
    m_hood.setSetpoint(setpoint);
  }

  //   public void goToSetpoint(Rotation2d wristSetpoint, Rotation2d elevatorSetpoint, Rotation2d jointSetpoint){
  //   m_wrist.setSetpoint(wristSetpoint);
  //   m_elevator.setSetpoint(elevatorSetpoint);
  //   m_joint.setSetpoint(jointSetpoint);
  // }

  // public void intake(){
  //   m_toaster.setState(ToasterState.INTAKE);
  //   m_indexer.setState(IndexerState.ON);
  // }

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
        wristGoToSetpoint(null);;
        elevatorGoToSetpoint(0);
        jointGoToSetpoint(null);
        m_indexer.setState(IndexerState.ON);
        m_toaster.setState(ToasterState.INTAKE);
        break;
      case AMP:
        //add amp assist subsystem
        //elevator out
        //joint up
        jointGoToSetpoint(null);
        elevatorGoToSetpoint(0);
        hoodGoToSetpoint(null);
        shoot(0);

        break;
      case SOURCE:
        //join up
        //elevator out
        jointGoToSetpoint(null);
        elevatorGoToSetpoint(0);
        m_toaster.setState(ToasterState.INTAKE);
        break;
      case SHOOT:
        //photonvisionnnn for aim assist
        jointGoToSetpoint(null);
        shoot(100);
        break;
      case TRAP:
        //wrist goes all the way up
        //trap mechanism activates
        wristGoToSetpoint(null);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
