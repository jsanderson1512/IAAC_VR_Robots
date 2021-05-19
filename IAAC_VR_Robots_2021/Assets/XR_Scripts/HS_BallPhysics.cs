using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;
using Photon.Pun.Demo.Cockpit;

public class HS_BallPhysics : MonoBehaviour
{
    private PhotonView photonView;
    public Vector3 throwSpeed = new Vector3(0, 3, 4);
    public int secondsToWait = 10;
    private bool destroyBall;
    private GameObject arAnchor;
    private Color myColor;
    private float scaleModifier = 1.0f;//starts at 1*current scale


    void Start()
    {
        if (photonView == null)
        {
            photonView = GetComponent<PhotonView>();
        }

        if (photonView != null)
        {
            if (photonView.IsMine)
            {
                //Debug.Log("hello, ball photon view is mine");

                //do normal, local physics
                //SEND RPC TO SET PROPER STARTING POSITION


                if (arAnchor == null)
                {
                    arAnchor = GameObject.FindGameObjectWithTag("AR_Anchor");
                }
                if (arAnchor != null)
                {
                    gameObject.transform.parent = arAnchor.transform;
                    myColor = new Color(Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f));

                    Vector3 sendColor = new Vector3(myColor.r, myColor.g, myColor.b);
                    Vector3 localPos = gameObject.transform.localPosition;
                    Quaternion localRot = gameObject.transform.localRotation;

                    //send local position, local rotation, and color
                    photonView.RPC("SendStartPosition", RpcTarget.AllBuffered, localPos, localRot, sendColor);


                    StartCoroutine(WaitToDie());
                    ShootBall();

                }


            }
        }
        else
        {
            //WAIT UNTIL RPC IS COMPLETE, THEN PERFORM THE SHOOT BALL
            //set parent object for myself, etc. BECAUSE IM NOT INSTANTIATED LOCALLY.
            //do rpc to send physics information starting point;
        }

    }
    

    public void ShootBall()
    {
        gameObject.GetComponent<MeshRenderer>().material.color = myColor;

        gameObject.GetComponent<Rigidbody>().AddRelativeForce(throwSpeed, ForceMode.Impulse);
    }

    IEnumerator WaitToDie()
    {
        //yield on a new YieldInstruction that waits for 5 seconds.
        yield return new WaitForSeconds(secondsToWait);

        destroyBall=true;
    }

    private void Update()
    {
        if (!destroyBall)
        {
            return;
        }
        else
        {
            StartCoroutine(LerpFunction(0.0f, 2.0f));
        }
    }


    IEnumerator LerpFunction(float endValue, float duration)
    {
        float time = 0;
        float startValue = scaleModifier;
        Vector3 startScale = transform.localScale;

        while (time < duration)
        {
            scaleModifier = Mathf.Lerp(startValue, endValue, time / duration);
            transform.localScale = startScale * scaleModifier;
            time += Time.deltaTime;
            yield return null;
        }
        PhotonNetwork.Destroy(gameObject);
    }

    [PunRPC]
    void SendStartPosition(Vector3 position, Quaternion rotation, Vector3 theColor)
    {
        if (photonView == null)
        {
            photonView = GetComponent<PhotonView>();
        }
        if (photonView != null)
        {
            if (photonView.IsMine)
            {
                //dont do anything
            }
            else
            {
                //Debug.Log("THIS MESSAGE WAS SENT FROM REMOTE PLAYER: " + photonView.ViewID);

                if (arAnchor == null)
                {
                    arAnchor = GameObject.FindGameObjectWithTag("AR_Anchor");
                }
                if (arAnchor != null)
                {
                    transform.parent = arAnchor.transform;
                    transform.localPosition = position;
                    transform.localRotation = rotation;
                    myColor = new Color(theColor.x,theColor.y,theColor.z);

                    StartCoroutine(WaitToDie());
                    ShootBall();
                }
            }
        }
    }
}
