using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Photon.Pun;

public class InClassDemo_SimpleScript : MonoBehaviour
{
    public GameObject myGameObject;
    private float xPosition;
    private bool thingActivated = false;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("hello, my game object is called: " + myGameObject.name);
    }

    // Update is called once per frame
    void Update()
    {
        if(thingActivated == true)
        {
            myGameObject.transform.position = new Vector3(myGameObject.transform.position.x,
                myGameObject.transform.position.y - 0.01f, myGameObject.transform.position.z);
        }



        if(Input.GetKeyDown(KeyCode.W))
        {

            this.gameObject.transform.position = new Vector3(this.gameObject.transform.position.x + 0.01f,
    this.gameObject.transform.position.y, this.gameObject.transform.position.z);

        }
        if (Input.GetKeyDown(KeyCode.A))
        {
            this.gameObject.transform.position = new Vector3(this.gameObject.transform.position.x,
this.gameObject.transform.position.y, this.gameObject.transform.position.z - 0.01f);
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            this.gameObject.transform.position = new Vector3(this.gameObject.transform.position.x - 0.01f,
  this.gameObject.transform.position.y, this.gameObject.transform.position.z);
        }
        if (Input.GetKeyDown(KeyCode.D))
        {
            this.gameObject.transform.position = new Vector3(this.gameObject.transform.position.x,
this.gameObject.transform.position.y, this.gameObject.transform.position.z + 0.01f);
        }
    }


    public void DoTheThing()
    {


        thingActivated = !thingActivated;

        Debug.Log("hello, you hit the special button");

    }

}
