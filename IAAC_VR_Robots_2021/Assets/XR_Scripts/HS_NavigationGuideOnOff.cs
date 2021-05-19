using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HS_NavigationGuideOnOff : MonoBehaviour
{
    bool navigationGuideSwitcher;
    public GameObject navigationPanel;

    public void NavigationGuideOnOff()
    {

        navigationGuideSwitcher = !navigationGuideSwitcher;

        if (navigationGuideSwitcher)
        {
            navigationPanel.SetActive(true);
        }

        if(!navigationGuideSwitcher)
        {
            navigationPanel.SetActive(false);
        }
    }    
}
