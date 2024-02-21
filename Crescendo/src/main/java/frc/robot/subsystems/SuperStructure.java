package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
  private static SuperStructure m_instance = new SuperStructure();

  public SuperStructure() {}

  public static SuperStructure getInstance(){
    if (m_instance == null){
      m_instance = new SuperStructure();
    }
    return m_instance;
  }

  private enum SuperStructureState{
    OFF,
    ZERO,
    GROUND,
    AMP,
    SOURCE,
    SHOOT,
    TRAP
  }

  private SuperStructureState m_state = SuperStructureState.OFF;

  public void setState(SuperStructureState newState) {
    m_state = newState;
    switch (m_state) {
      case OFF:
        break;
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
