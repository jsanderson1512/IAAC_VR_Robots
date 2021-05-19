using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Photon.Pun;

public class HS_LoadScene : MonoBehaviour
{
    public string SceneToLoad;

    public void LoadThisScene()
    {
        StartCoroutine(DoSwitchLevel(SceneToLoad));
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