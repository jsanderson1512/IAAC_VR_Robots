using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HS_PlayerPositionWorldToLocal : MonoBehaviour
{

    private GameObject arCamera;
    public string tagToFind;


    void Update()
    {

        if( arCamera == null)
        {
            arCamera = GameObject.FindGameObjectWithTag(tagToFind);
        }



        if (arCamera != null)
        {
            transform.position = arCamera.transform.position;
            transform.rotation = arCamera.transform.rotation;
        }       
    }
}
