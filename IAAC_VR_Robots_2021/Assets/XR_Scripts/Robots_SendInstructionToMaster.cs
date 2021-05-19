using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon;
using Photon.Pun;


public class Robots_SendInstructionToMaster : MonoBehaviour
{
    public GameObject RobotTarget_RobotSpaceUnityUnits;

    public void SendToMaster()
    {
        GameObject[] myPlayers = GameObject.FindGameObjectsWithTag("Player");

        foreach (GameObject g in myPlayers)
        {
            PhotonView PV = g.GetComponent<PhotonView>();
            if(PV.IsMine)
            {
                Robots_SendInstructionsRPC myRPC = g.GetComponent<Robots_SendInstructionsRPC>();
                if(myRPC)
                {
                    myRPC.SendInstructions(RobotTarget_RobotSpaceUnityUnits.transform.localPosition, 
                        RobotTarget_RobotSpaceUnityUnits.transform.right, RobotTarget_RobotSpaceUnityUnits.transform.forward);
                }
            }
        }
    }

    public void SlavePosFromMaster(Vector3 thePos, Quaternion theRot)
    {
        GameObject[] myPlayers = GameObject.FindGameObjectsWithTag("Player");

        foreach (GameObject g in myPlayers)
        {
            PhotonView PV = g.GetComponent<PhotonView>();
            if (PV.IsMine)
            {
                Robots_SendInstructionsRPC myRPC = g.GetComponent<Robots_SendInstructionsRPC>();
                if (myRPC)
                {
                    myRPC.SlavePosFromMaster(thePos, theRot);
                }
            }
        }
    }
}
