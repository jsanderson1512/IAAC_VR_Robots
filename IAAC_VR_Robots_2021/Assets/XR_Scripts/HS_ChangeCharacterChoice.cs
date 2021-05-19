using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HS_ChangeCharacterChoice : MonoBehaviour
{
    private GameObject userChoiceObject;


    public void ChangeChoice(int charIndex)
    {
        userChoiceObject = GameObject.FindGameObjectWithTag("AR_CharacterChoice");
        userChoiceObject.GetComponent<HS_CharacterChoice>().characterIndex = charIndex;

    }
}
