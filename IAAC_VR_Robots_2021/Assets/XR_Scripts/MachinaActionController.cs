using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MachinaActionController : MonoBehaviour
{
    private ConnectToMachina MachinaConnect;
    public GameObject RobotTarget_RobotSpaceUnityUnits;
    public GameObject myRobot;

    private void Awake()
    {
        MachinaConnect = GameObject.FindGameObjectWithTag("GameManager").GetComponent<ConnectToMachina>();
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    public void GoToRobotTarget()
    {

        Vector3 robotPositionUnity = RobotTarget_RobotSpaceUnityUnits.transform.localPosition;
        Vector3 robotPositionBridge = new Vector3(-1000*robotPositionUnity.x,-1000*robotPositionUnity.z, 1000*robotPositionUnity.y);

        //this is in world space - doesnt work if robot is rotated
        //Vector3 right = RobotTarget_RobotSpaceUnityUnits.transform.right;
        //Vector3 back = -RobotTarget_RobotSpaceUnityUnits.transform.forward;

        Vector3 rightInRobotWorld = myRobot.transform.InverseTransformDirection(RobotTarget_RobotSpaceUnityUnits.transform.right);
        Vector3 forwardInRobotWorld = myRobot.transform.InverseTransformDirection(RobotTarget_RobotSpaceUnityUnits.transform.forward);

        Vector3 rightBridge = new Vector3(rightInRobotWorld.x, rightInRobotWorld.z, -1 * rightInRobotWorld.y);
        Vector3 forwardBridge = new Vector3(forwardInRobotWorld.x,  forwardInRobotWorld.z, -1 * forwardInRobotWorld.y);
        //this is repeated in the RPC command below

        Debug.Log("right bridge is: " + rightBridge);
        Debug.Log("forward bridge is: " + forwardBridge);


#if UNITY_STANDALONE_WIN

        //MachinaConnect.MoveTo(robotPositionBridge.x, robotPositionBridge.y, robotPositionBridge.z);
        MachinaConnect.TransformTo(robotPositionBridge.x, robotPositionBridge.y, robotPositionBridge.z,
            rightBridge.x, rightBridge.y, rightBridge.z, forwardBridge.x, forwardBridge.y, forwardBridge.z);
        #endif

    }

    public void GoToRobotTargetRPC(Vector3 localPos, Vector3 right, Vector3 forward)
    {

        Vector3 robotPositionBridge = new Vector3(-1000 * localPos.x, -1000 * localPos.z, 1000 * localPos.y);


        Vector3 rightInRobotWorld = myRobot.transform.InverseTransformDirection(right);
        Vector3 forwardInRobotWorld = myRobot.transform.InverseTransformDirection(forward);

        Vector3 rightBridge = new Vector3(rightInRobotWorld.x, rightInRobotWorld.z, -1 * rightInRobotWorld.y);
        Vector3 forwardBridge = new Vector3(forwardInRobotWorld.x, forwardInRobotWorld.z, -1 * forwardInRobotWorld.y);

#if UNITY_STANDALONE_WIN

        //MachinaConnect.MoveTo(robotPositionBridge.x, robotPositionBridge.y, robotPositionBridge.z);
        MachinaConnect.TransformTo(robotPositionBridge.x, robotPositionBridge.y, robotPositionBridge.z,
            rightBridge.x, rightBridge.y, rightBridge.z, forwardBridge.x, forwardBridge.y, forwardBridge.z);
        #endif

    }


    private void Update()
    {
        //here you can add keyboard input
        /*
        if (Input.GetKeyDown(KeyCode.W))
        {
            MachinaConnect.Move(100, 0, 0);
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            MachinaConnect.Move(-100, 0, 0);
        }
        if (Input.GetKeyDown(KeyCode.A))
        {
            MachinaConnect.Move(0, 100, 0);
        }
        if (Input.GetKeyDown(KeyCode.D))
        {
            MachinaConnect.Move(0, -100, 0);
        }
        if (Input.GetKeyDown(KeyCode.Q))
        {
            MachinaConnect.Move(0, 0, 100);
        }
        if (Input.GetKeyDown(KeyCode.E))
        {
            MachinaConnect.Move(0, 0, -100);
        }
        */

        if (Input.GetKeyDown(KeyCode.Space))
        {
            GoToRobotTarget();
        }
  
    }
}
