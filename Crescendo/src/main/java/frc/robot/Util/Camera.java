package frc.robot.Util;

import org.photonvision.PhotonCamera;

public class Camera {
    private PhotonCamera m_camera1;
    private var results;
    private boolean hasTargets;

    public Camera(String cameraName){
        m_camera1 = new PhotonCamera(cameraName);
    }

    
    public void periodic() {
        
    }
}
