using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HS_CharacterChoice : MonoBehaviour
{



    private static HS_CharacterChoice _instance;

    public static HS_CharacterChoice Instance { get { return _instance; } }


    private void Awake()
    {
        if (_instance != null && _instance != this)
        {
            Destroy(this.gameObject);
        }
        else
        {
            _instance = this;
            DontDestroyOnLoad(_instance);
        }
    }


    private void Start()
    {
        this.gameObject.tag = "AR_CharacterChoice";
    }


    public int characterIndex = 0;



    public void ChangeCharacterIndex(int index)
    {
        if (index == 0)
            characterIndex = 0;
        if (index == 1)
            characterIndex = 1;
        if (index == 2)
            characterIndex = 2;
        if (index == 3)
            characterIndex = 3;

    }

    
}
