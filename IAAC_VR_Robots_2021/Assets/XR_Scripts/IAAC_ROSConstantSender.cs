using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class IAAC_ROSConstantSender : MonoBehaviour
{
    public Transform FKTcpPos;
    private Transform PosToTarget;

    public RosSharp_Kinematics IKModel;

    public float tolerance = 0.05f;
    public float waitForSeconds = 1.0f;
    private bool movingNow;

    public Image toggleButtonImage;

    private bool movingActivated;

    // Update is called once per frame
    void Update()
    {

        PosToTarget = IKModel.RobotTarget_RobotSpace;

        if (movingActivated)
        {
            if (!movingNow)
            {
                if (Vector3.Distance(FKTcpPos.position, PosToTarget.position) > tolerance)
                {
                    StartCoroutine(waitAfterSending(waitForSeconds));
                    IKModel.SendPointsToROS();
                    movingNow = true;
                }
            }

        }



    }
    IEnumerator waitAfterSending(float howLong)
    {

        yield return new WaitForSeconds(howLong);
        movingNow = false;

    }

    public void ToggleFollower()
    {
        movingActivated = !movingActivated;

        if (movingActivated)
        {
            toggleButtonImage.color = Color.green;

        }
        else
        {
            toggleButtonImage.color = Color.yellow;

        }
    }
}
