using Photon.Pun;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class HS_PlayerInfo : MonoBehaviourPun
{
    public TextMeshPro playerNameDisplay;
    // Start is called before the first frame update
    void Start()
    {
        if(!photonView.IsMine)
        {
            playerNameDisplay.text = photonView.Owner.NickName ;
        }      
    }
}
