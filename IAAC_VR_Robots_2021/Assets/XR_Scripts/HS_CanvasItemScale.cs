using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HS_CanvasItemScale : MonoBehaviour
{
    private float scaleFactor = 1125.0f;
    private float scale;

    private void Start()
    {
        scale = Screen.height / scaleFactor;
        transform.GetComponent<RectTransform>().localScale = new Vector3(scale, scale, scale);
    }
}