using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HS_DontSleep : MonoBehaviour
{
    void Start()
    {
        Screen.sleepTimeout = SleepTimeout.NeverSleep;
    }
}
