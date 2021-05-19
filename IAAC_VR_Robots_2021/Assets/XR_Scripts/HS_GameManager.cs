using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Photon.Pun;
using Photon.Realtime;
using Photon.Pun.Demo.PunBasics;
using UnityEngine.SceneManagement;

public class HS_GameManager : MonoBehaviour
{
    public GameObject firstPersonCamera;
    public Text roomNameDisplay;
    public GameObject ARPrefabFromResources;
    public GameObject VRPrefabHeadFromResources;
    public GameObject VRPrefabControllerLFromResources;
    public GameObject VRPrefabControllerRFromResources;

    public string connectSceneName;
    private GameObject userCharacterChoice;

    // Start is called before the first frame update
    void Start()
    {
        if (!PhotonNetwork.IsConnected)
        {
            SceneManager.LoadScene(connectSceneName);
            return;
        }

        roomNameDisplay.text = PhotonNetwork.CurrentRoom.Name;
        //roomnamedisplay += " ip address";

        if (PlayerManager.LocalPlayerInstance == null)
        {
            //PhotonNetwork.Instantiate(ARPrefabFromResources.name, Vector3.zero, Quaternion.identity, 0);

            //need to say if we are in vr do the vr prefab, else do the ar prefab

#if UNITY_STANDALONE_WIN



            if (firstPersonCamera.GetComponent<SteamVR_Camera>().enabled)
            {
                PhotonNetwork.Instantiate(VRPrefabHeadFromResources.name, Vector3.zero, Quaternion.identity, 0);
                PhotonNetwork.Instantiate(VRPrefabControllerLFromResources.name, Vector3.zero, Quaternion.identity, 0);
                PhotonNetwork.Instantiate(VRPrefabControllerRFromResources.name, Vector3.zero, Quaternion.identity, 0);

                //spawned controllers and headset will FIND VR camera parent objects
            }
            else
            {
                        PhotonNetwork.Instantiate(ARPrefabFromResources.name, Vector3.zero, Quaternion.identity, 0);

            }
            
#else
            PhotonNetwork.Instantiate(ARPrefabFromResources.name, Vector3.zero, Quaternion.identity, 0);


#endif

        }

    }

    
    private void Update()
    {

        
        if (!PhotonNetwork.IsConnected)
        {
            roomNameDisplay.text = "Not Connected";



            //SceneManager.LoadScene("HS_LoadingScene");
            //return;
        }
        else
        {
            PhotonNetwork.CurrentRoom.IsOpen = true;
            PhotonNetwork.CurrentRoom.IsVisible = true;
            //Debug.Log("my player count is: " + PhotonNetwork.CurrentRoom.PlayerCount);
        }
    }



    public void LoadThisScene()
    {
        StartCoroutine(DoSwitchLevel(connectSceneName));
    }
    IEnumerator DoSwitchLevel(string level)
    {
        if (PhotonNetwork.IsConnected)
        {
            PhotonNetwork.Disconnect();
            while (PhotonNetwork.IsConnected)
                yield return null;
        }

        SceneManager.LoadScene(level);
    }

}
