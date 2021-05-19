using Photon.Pun;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class HS_PlayerPositionUpdater : MonoBehaviour
{
    private PhotonView photonView;
    public string worldToLocalPositionToFollow = "AR_LocalPosition";

    private GameObject arLocalPosition;
    private GameObject arAnchor;

    private Vector3 lastPos;
    private Quaternion lastRot;
    private float distanceTolerance = 0.0254f; //one inch in meters
    private float rotationTolerance = 1.0f;//one degree of difference between two quaternions


    private void Update()
    {

        if (photonView == null)
        {
            photonView = GetComponent<PhotonView>();
        }

        if (photonView != null)
        {
            if (photonView.IsMine)
            {
                MeshRenderer[] myPlayerRenderer = transform.gameObject.GetComponentsInChildren<MeshRenderer>();

                foreach (MeshRenderer mr in myPlayerRenderer)
                {
                    mr.enabled = false;
                }

                if (arLocalPosition == null)
                {
                    arLocalPosition = GameObject.FindGameObjectWithTag(worldToLocalPositionToFollow);
                }
                if (arLocalPosition != null)
                {

                    Vector3 localPos = arLocalPosition.transform.localPosition;
                    Quaternion localRot = arLocalPosition.transform.localRotation;

                    float currentDifference = Vector3.Distance(localPos, lastPos);
                    float currentRotation = Quaternion.Angle(localRot, lastRot);

                    //Debug.Log("rotation difference is..." + currentRotation);

                    if(lastPos != localPos || lastRot != localRot)
                    {
                        if (currentDifference > distanceTolerance || currentRotation > rotationTolerance)
                        {
                            photonView.RPC("UpdateRemotePlayerPosition", RpcTarget.AllBuffered, localPos, localRot);
                            lastPos = localPos;
                            lastRot = localRot;
                        }
                    }
                }
            }
            else
            {
                //this controls if you are non-local player and the session you are in doesn't have a aranchor yet...

                if (arAnchor == null)
                {
                    arAnchor = GameObject.FindGameObjectWithTag("AR_Anchor");
                }
                if (arAnchor == null)
                {
                    MeshRenderer[] myPlayerRenderer = transform.gameObject.GetComponentsInChildren<MeshRenderer>();

                    foreach (MeshRenderer mr in myPlayerRenderer)
                    {
                        mr.enabled = false;
                    }
                }
                else
                {
                    MeshRenderer[] myPlayerRenderer = transform.gameObject.GetComponentsInChildren<MeshRenderer>();

                    foreach (MeshRenderer mr in myPlayerRenderer)
                    {
                        mr.enabled = true;
                    }
                }
            }
        }
    }

    [PunRPC]
    void UpdateRemotePlayerPosition(Vector3 position, Quaternion rotation)
    {
        if (photonView == null)
        {
            photonView = GetComponent<PhotonView>();
        }
        if(photonView != null)
        {
            if (photonView.IsMine)
            {

            }
            else
            {
                //Debug.Log("THIS MESSAGE WAS SENT FROM REMOTE PLAYER: " + photonView.ViewID);

                if (arAnchor == null)
                {
                    arAnchor = GameObject.FindGameObjectWithTag("AR_Anchor");
                }
                if(arAnchor != null)
                {
                    transform.parent = arAnchor.transform;
                    transform.localPosition = position;
                    transform.localRotation = rotation;
                }

            }
        }
    }

}
