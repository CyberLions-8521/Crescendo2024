package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Toaster.ToasterState;
import frc.robot.subsystems.Wrist.WristState;

public class SuperStructure extends SubsystemBase {

  public static final Wrist m_wrist = new Wrist();
  public static final Elevator m_elevator = new Elevator();
  public static final Indexer m_indexer = new Indexer();
  public static final Toaster m_toaster = new Toaster();

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
  }

  public void shoot(double RPM/* */){
    //m_joint.goToSetpoint();
    m_toaster.setRPM(RPM);
  }

  public void wristGoToSetpoint(Rotation2d setpoint){
    m_wrist.setSetpoint(setpoint);
  }
  public void elevatorGoToSetpoint(Rotation2d setpoint){
    m_elevator.setSetpoint(setpoint);
  }

  public void setState(SuperStructureState newState) {
    m_state = newState;
    switch (m_state) {
      case ZERO:
        zeroAll();
        break;
      case GROUND:
        //wrist comes out
        wristGoToSetpoint(Rotation2d.fromRotations(0));
        //elevator comes out a bit
        elevatorGoToSetpoint(Rotation2d.fromRotations(0));
        m_toaster.setState(ToasterState.INTAKE);
        m_indexer.setState(IndexerState.ON);
        break;
      case AMP:
        break;
      case SOURCE:
        break;
      case SHOOT:
        //photonvisionnnn
        shoot(100);
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
