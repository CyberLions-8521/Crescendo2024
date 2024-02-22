package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;

public class SuperStructure extends SubsystemBase {

  public static final Wrist m_wrist = new Wrist();
  public static final Elevator m_elevator = new Elevator();
  public static final Indexer m_indexer = new Indexer();
  public static final Toaster m_toaster = new Toaster();

  public SuperStructure() {}

  public void zeroAll(){
    m_wrist.setState(WristState.ZERO);
    m_elevator.setState(ElevatorState.ZERO);
  }

  private enum SuperStructureState{
    // OFF,
    ZERO,
    GROUND,
    AMP,
    SOURCE,
    SHOOT,
    TRAP
  }

  public void zeroALL(){

  }

  private SuperStructureState m_state = SuperStructureState.ZERO;

  public void setState(SuperStructureState newState) {
    m_state = newState;
    switch (m_state) {
      // case OFF:
      //   break;
      case ZERO:
        break;
      case GROUND:
        break;
      case AMP:
        break;
      case SOURCE:
        break;
      case SHOOT:
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
