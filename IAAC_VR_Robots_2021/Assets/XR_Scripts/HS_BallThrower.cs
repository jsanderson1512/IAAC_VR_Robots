using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;
using UnityEngine.EventSystems;

public class HS_BallThrower : MonoBehaviour
{
    private GameObject ballToThrow;
    private GameObject arAnchor;

    // Update is called once per frame
    void Update()
    {
        if (arAnchor == null)
        {
            arAnchor = GameObject.FindGameObjectWithTag("AR_Anchor");
        }
        if (arAnchor != null)
        {
            if (!IsPointerOverUIObject())
            {
                if (Input.GetMouseButtonDown(0))
                {
                    ThrowBall();
                }
                else if (Input.touchCount > 0 && Input.touches[0].phase == TouchPhase.Began)
                {
                    ThrowBall();
                }
            }
        }
    }

    /// <summary>
    /// used to prevent raycasting when clicking on UI elements
    /// </summary>
    /// <returns></returns>
    private bool IsPointerOverUIObject()
    {
        PointerEventData eventDataCurrentPosition = new PointerEventData(EventSystem.current);
        eventDataCurrentPosition.position = new Vector2(Input.mousePosition.x, Input.mousePosition.y);
        List<RaycastResult> results = new List<RaycastResult>();
        EventSystem.current.RaycastAll(eventDataCurrentPosition, results);
        return results.Count > 0;
    }

    public void ThrowBall()
    {
        Vector3 instantiatePosition = gameObject.transform.position + gameObject.transform.forward * 0.4f;


        //this is for single player
        //ballToThrow = Instantiate(prefab_BallToThrow, instantiatePosition, gameObject.transform.rotation);

        ballToThrow = PhotonNetwork.Instantiate("Prefab_BallToThrow",
                                instantiatePosition, 
                                gameObject.transform.rotation);


        ballToThrow.transform.parent = arAnchor.transform;
    }
}
