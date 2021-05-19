using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;

public class Robots_SendInstructionsRPC : MonoBehaviour
{
    private PhotonView photonView;

    public void SendInstructions(Vector3 localPos, Vector3 right, Vector3 forward)
    {
        if (photonView == null)
        {
            photonView = GetComponent<PhotonView>();
        }
        if (photonView != null)
        {
            if (photonView.IsMine)
            {
                photonView.RPC("SendToGameManager", RpcTarget.AllBuffered, localPos, right, forward);

            }
        }
    }

    public void SlavePosFromMaster(Vector3 thePos, Quaternion theRot)
    {
        if (photonView == null)
        {
            photonView = GetComponent<PhotonView>();
        }

        if (photonView != null)
        {
            if (photonView.IsMine)
            {
                photonView.RPC("SendSlavePosGameManager", RpcTarget.AllBuffered, thePos, theRot);

            }
        }
    }

    [PunRPC]
    void SendToGameManager(Vector3 localPos, Vector3 right, Vector3 forward)
    {
        GameObject SceneManager = GameObject.FindGameObjectWithTag("GameManager");
        if (SceneManager)
        {
            MachinaActionController MST = SceneManager.GetComponent<MachinaActionController>();
            if(MST)
            {
                MST.GoToRobotTargetRPC(localPos, right, forward);
            }
        }
    }

    [PunRPC]
    void SendSlavePosGameManager(Vector3 thePos, Quaternion theRot)
    {
        GameObject SceneManager = GameObject.FindGameObjectWithTag("GameManager");
        if (SceneManager)
        {
            ConnectToMachina CTM = SceneManager.GetComponent<ConnectToMachina>();
            if (CTM)
            {
                CTM.MoveSlaveToPosition(thePos, theRot);
            }
        }
    }
}
