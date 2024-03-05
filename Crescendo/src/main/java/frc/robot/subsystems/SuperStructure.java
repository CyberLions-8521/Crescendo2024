package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Hood.HoodState;
import frc.robot.subsystems.HoodWrist.HoodWristState;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Joint.JointState;
import frc.robot.subsystems.Toaster.ToasterState;
import frc.robot.subsystems.IndexerWrist.IndexerWristState;

public class SuperStructure extends SubsystemBase {
  public final Elevator m_elevator = Elevator.getInstance();
  public final Hood m_hood = Hood.getInstance();
  public final Indexer m_indexer = Indexer.getInstance();
  public final Toaster m_toaster = Toaster.getInstance();

  //WRISTS
  public final IndexerWrist m_indexerWrist = IndexerWrist.getInstance();
  public final HoodWrist m_hoodWrist = HoodWrist.getInstance();
  public final Joint m_joint = Joint.getInstance();

  //Controller
  // public final CommandXboxController m_driverController;


  private SuperStructure(/*Toaster m_toaster, Elevator m_elevator, Joint m_joint, HoodWrist m_hoodWrist/*, Hood m_hood, Indexer m_indexer, IndexerWrist m_indexerWrist , CommandXboxController m_driverController*/) {
  //   this.m_toaster = m_toaster;
  //   this.m_elevator = m_elevator;
  //   this.m_joint = m_joint;
  //   this.m_hoodWrist = m_hoodWrist;
  //   this.m_hood = m_hood;
  //   this.m_indexer = m_indexer;
  //   this.m_indexerWrist = m_indexerwrist;
  //   this.m_driverController = m_driverController;
  }

  private static SuperStructure m_instance = new SuperStructure();

  public static SuperStructure getInstance(){
    return m_instance;
  }

  public enum SuperStructureState{
    OFF,
    ZERO,
    GROUND_INTAKE,
    SOURCE,
    SPEAKER_SHOOT,
    AMP_SHOOT,
    TRAP
  }

  // private static SuperStructure m_instance = new SuperStructure();

  // public static SuperStructure getInstance(){
  //   return m_instance;
  // }
  
  private SuperStructureState m_state = SuperStructureState.ZERO;

  public void offALL(){
    m_toaster.setSpeed(0, 0);
    m_elevator.set(0);
  }
  
  public void zeroAll(){
    //WRISTS
    //joint is zerod only for ground intake isn't it? bc limit swithc is at the bottom
    // m_joint.setState(JointState.ZERO);
    // m_indexerWrist.setState(IndexerWristState.ZERO);
    // m_hoodWrist.setState(HoodWristState.ZERO);
    //REGULAR SUBSYSTEM
    m_elevator.setState(ElevatorState.ZERO);
  }

  public void goToElevatorPosition(Rotation2d jointSetpoint, Double elevatorSetpoint){
    // m_joint.setSetpoint(jointSetpoint);
    m_elevator.setSetpoint(elevatorSetpoint);
  }

  public void hoodGoToSetpoint(double hoodSetpoint){
    m_hoodWrist.setSetpoint(hoodSetpoint);
  }

  public void setState(SuperStructureState newState) {
    m_state = newState;

    switch (m_state) {
      case OFF:
        offALL();
        break;

      case ZERO:
        zeroAll();
        break;

      case GROUND_INTAKE:
        // goToElevatorPosition(0,0);
        // m_indexerWrist.setSetpoint(0);
        // m_indexer.setState(IndexerState.ON);
        m_toaster.setState(ToasterState.INTAKE);
        // m_hoodWrist.setState(ToasterState.ZERO);
        break;
    
      case SOURCE:
        // goToElevatorPosition(0,0);
        // m_hood.setState(HoodState.OFF);
        m_toaster.setState(ToasterState.INTAKE);
        break;
      
      case SPEAKER_SHOOT:
        // jointGoToSetpoint(null);
        m_toaster.setState(ToasterState.SPEAKER_SHOOT);
        // m_hood.setState(HoodState.OFF);
        
        break;
        
      case AMP_SHOOT:
        // jointGoToSetpoint(null);
        // elevatorGoToSetpoint(null);
        hoodGoToSetpoint(0);
        // m_hood.setState(HoodState.ON);
        m_toaster.setState(ToasterState.AMP_SHOOT);

      case TRAP:
        // jointGoToSetpoint(null);
        // elevatorGoToSetpoint(null);
        
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
