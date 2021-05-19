using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;
using UnityEngine.UI;

public class HS_BallCounter : MonoBehaviour
{
    private PhotonView photonView;
    public Text CounterText;
    private int counter = 0;

    private void Start()
    {
        CounterText.text = counter.ToString();


    }
    private void OnTriggerEnter(Collider other)
    {
        photonView = other.GetComponent<PhotonView>();

        if (photonView != null)
        {
            if (photonView.IsMine)
            {
                counter++;
                Mathf.Clamp(counter, 0f, 1000f);
                CounterText.text = counter.ToString();

            }
        }
    }
}
