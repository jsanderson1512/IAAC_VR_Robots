using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class HS_CameraPlatformManager : MonoBehaviour
{
    [Header ("WebGL Stuff")] 
    public GameObject WebGLCameraParent;
    public GameObject WebGLCanvasParent;

    [Header("AR Stuff")]
    public GameObject ARCameraParent;
    public GameObject ARCanvasParent;
    //public GameObject iOSShadowPlane;

    [Header("VR Stuff")]
    public GameObject VRCameraParent;
    public GameObject VRCanvasParent;
    public GameObject firstPersonCamera;
    //public GameObject iOSShadowPlane;

    [Header("Geometry")]
    public GameObject ARObject;

    private void Awake()
    {

#if UNITY_STANDALONE_WIN
        Debug.Log("Stand Alone Windows");
        VRCameraParent.SetActive(true);

        VRCanvasParent.SetActive(true);
        WebGLCanvasParent.SetActive(false);
        ARCameraParent.SetActive(false);
        ARCanvasParent.SetActive(false);
        ARObject.SetActive(true);

        if (firstPersonCamera.GetComponent<SteamVR_Camera>().enabled)
        {
            //activate VR cam
            WebGLCameraParent.SetActive(false);
            VRCameraParent.SetActive(true);
        }
        else
        {
            //activate WebGL cam
            WebGLCameraParent.SetActive(true);
            VRCameraParent.SetActive(false);
        }

#elif UNITY_WEBGL
        Debug.Log("WebGL");


            //activate WebGL mode

            WebGLCameraParent.SetActive(true);
            WebGLCanvasParent.SetActive(true);

            ARCameraParent.SetActive(false);
            ARCanvasParent.SetActive(false);
            //iOSShadowPlane.SetActive(false);

            VRCameraParent.SetActive(false);
            VRCanvasParent.SetActive(false);

            ARObject.SetActive(true);

#elif UNITY_IOS
        Debug.Log("Iphone");

        WebGLCameraParent.SetActive(false);
        WebGLCanvasParent.SetActive(false);

        ARCameraParent.SetActive(true);
        ARCanvasParent.SetActive(true);
        //iOSShadowPlane.SetActive(true);

        VRCameraParent.SetActive(false);
        VRCanvasParent.SetActive(false);

        ARObject.SetActive(false);

#elif UNITY_ANDROID

//inside of unity_android we will need to differentiate between oculus and mobile android...

        Debug.Log("Iphone");

        WebGLCameraParent.SetActive(false);
        WebGLCanvasParent.SetActive(false);

        ARCameraParent.SetActive(true);
        ARCanvasParent.SetActive(true);
        //iOSShadowPlane.SetActive(true);

        VRCameraParent.SetActive(false);
        VRCanvasParent.SetActive(false);

        ARObject.SetActive(false);


#else
        Debug.Log("Any Other Platform");

#endif



    }
}
