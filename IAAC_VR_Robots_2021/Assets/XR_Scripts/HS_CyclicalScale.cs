using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HS_CyclicalScale : MonoBehaviour
{

    private float currentScale;
    private float newScale;
    public float speed = 25.0f;
    public float amplitude = 1.0f;
    // Start is called before the first frame update
    void Start()
    {
        currentScale = transform.GetComponent<RectTransform>().localScale.x;
    }

    // Update is called once per frame
    void Update()
    {
        newScale = currentScale + (Mathf.Sin(Time.time * speed) * amplitude);
        transform.GetComponent<RectTransform>().localScale = new Vector3(newScale, newScale, newScale);
    }
}
