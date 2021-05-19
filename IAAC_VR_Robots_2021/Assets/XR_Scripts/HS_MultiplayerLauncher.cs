using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using Photon.Pun;
using Photon.Realtime;

using System.Linq;

public class HS_MultiplayerLauncher : MonoBehaviourPunCallbacks, ILobbyCallbacks
{
    //[SerializeField]
    //private GameObject controlPanel;

    [Header("Game Info")]
    public string gameVersion = "0.0.1";
    public string SceneToLoad;
    public string multiplayerRoomName;
    public int maxPlayersPerRoom = 10;


    [Header("UI Info")]
    public InputField userNameInput;
    public Text statusLog;
    public GameObject roomSetupUI;
    public GameObject joinRoomButton;
    public static string userName;
    private int roomCounter = 0;
    private RoomOptions roomOptions = new RoomOptions();


    private void Awake()
    {
        //PhotonNetwork.AutomaticallySyncScene = true;

        //turned this off to avoid creating a duplicate scene for people.
    }


    private void Start()
    {
        userName = "";

        if (PlayerPrefs.HasKey("myPlayerName"))
        {
            userName = PlayerPrefs.GetString("myPlayerName");
            Debug.Log("I have a player name and it is: " + userName);
        }
        userNameInput.text = userName;

        print("CONNECTING...");

        roomSetupUI.SetActive(false);
        ConnectToPhoton();
    }

    //this is assigned to the OnValueChanged of the userName input field
    public void SetUserName()
    {
        userName = userNameInput.text;
        PlayerPrefs.SetString("myPlayerName", userName);
    }



    //When JoinRoom is clicked, check if the name field is empty first
    public void JoinRoomClicked()
    {
        if (!string.IsNullOrWhiteSpace(userNameInput.text))
        {
            JoinRoom();
        }
        else
        {
            statusLog.color = Color.white;
            statusLog.text = "Name field can not be empty!";
        }
    }



    #region NETWORK METHODS
    public void ConnectToPhoton()
    {
        statusLog.text = "Connecting to network...";
        PhotonNetwork.GameVersion = gameVersion;
        PhotonNetwork.OfflineMode = false;
        PhotonNetwork.ConnectUsingSettings();
        print("I just CONNECTED to network");
    }

    public void JoinRoom()
    {
        if (PhotonNetwork.IsConnected)
        {
            //set the player name
            PhotonNetwork.LocalPlayer.NickName = userName;
            roomOptions.MaxPlayers = (byte)maxPlayersPerRoom;
            roomOptions.IsOpen = true;
            TypedLobby myLobby = new TypedLobby(multiplayerRoomName + roomCounter.ToString(), LobbyType.Default);
            PhotonNetwork.JoinOrCreateRoom(multiplayerRoomName + roomCounter.ToString(), roomOptions, myLobby, null);            
        }
    }


    public void LoadScene()
    {
        if (PhotonNetwork.CurrentRoom.PlayerCount > 0)
        {
            PhotonNetwork.LoadLevel(SceneToLoad);
            print("I just LOADED the scene");
        }
    }

   #endregion

    #region CALLBACK OVERRIDES
    public override void OnConnected()
    {
        base.OnConnected();

        
        statusLog.color = Color.green;
        statusLog.text = "Connected";

        //turn on room joining UI
        roomSetupUI.SetActive(true);

    }

    public override void OnDisconnected(DisconnectCause cause)
    {
        //controlPanel.SetActive(true);
        statusLog.color = Color.white;
        statusLog.text = "Disconnected from network due to: " + cause.ToString();
    }

    public override void OnJoinedRoom()
    {
        print("Room JOINING successful");
        print(PhotonNetwork.CurrentRoom.Name);

        //load the scene here since OnJoinedRoom get called on both JOIN and CREATE
        LoadScene();
    }

    public override void OnCreatedRoom()
    {
        joinRoomButton.SetActive(false);

        statusLog.color = Color.green;
        statusLog.text = "You are the host of this room";

        PhotonNetwork.CurrentRoom.IsOpen = true;
        print("Room CREATION successful");

    }

    public override void OnCreateRoomFailed(short returnCode, string message)
    {
        statusLog.color = Color.red;
        statusLog.text = "Room was not created. " + message;
        Debug.Log("I failed to create room: " + multiplayerRoomName + roomCounter.ToString());
        Debug.Log("I failed to create room because: " + message);

        print(message);
    }

    public override void OnJoinRoomFailed(short returnCode, string message)
    {
        //if room joining fails then create the next one in the sequence 
        Debug.Log("I failed to join room: " + multiplayerRoomName + roomCounter.ToString());
        Debug.Log("I failed to join room because: " + message);

        roomCounter += 1;
        roomOptions.MaxPlayers = (byte)maxPlayersPerRoom;
        roomOptions.IsOpen = true;
        TypedLobby myLobby = new TypedLobby(multiplayerRoomName + roomCounter.ToString(), LobbyType.Default);
        PhotonNetwork.JoinOrCreateRoom(multiplayerRoomName + roomCounter.ToString(), roomOptions, myLobby, null);
    }
    #endregion
}
