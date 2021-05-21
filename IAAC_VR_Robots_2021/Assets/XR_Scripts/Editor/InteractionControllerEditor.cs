 using UnityEngine;
 using UnityEditor;
using System.Linq;

[CustomEditor(typeof(InteractionController))] //fix this name after class
public class InteractionControllerEditor : Editor
{
    //!! rename this "InteractionControllerEditor" after class

    private int previousOnCount;
    private int previousOffCount;

    public override void OnInspectorGUI()
    {
        InteractionController script = (InteractionController)target;


        //highlight material if raycast is selected!
        if (script.gameObject.GetComponent<Collider>().isTrigger == false)
        {
            if (script.HighlightMaterial == null)
            {
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = Color.cyan;
                defaultMaterial.name = "Color Cyan";

                script.HighlightMaterial = (Material)EditorGUILayout.ObjectField("Highlight Material", defaultMaterial, typeof(Material), true);
            }
            else
            {
                script.HighlightMaterial = (Material)EditorGUILayout.ObjectField("Highlight Material", script.HighlightMaterial, typeof(Material), true);
            }
        }



        script.myType = (InteractionController.InteractionType)EditorGUILayout.EnumPopup("Interaction Type", script.myType);



        if (script.myType == InteractionController.InteractionType.AnimationController)
        {
            script.ObjectWithAnimation = (GameObject)EditorGUILayout.ObjectField("Object with Animation", script.ObjectWithAnimation, typeof(GameObject),true);
        }
        else if (script.myType == InteractionController.InteractionType.SceneChangeController)
        {
            script.SceneToLoad = EditorGUILayout.TextField("Scene to Load", script.SceneToLoad);
        }
        else if (script.myType == InteractionController.InteractionType.OnOffController)
        {
            script.NumberOfThingsToTurnOn = EditorGUILayout.IntField("Number of Things to Turn On", script.NumberOfThingsToTurnOn);
            GameObject[] tempOns = script.ClickTurnsTheseON;
            if (previousOnCount != script.NumberOfThingsToTurnOn)
            {
                script.ClickTurnsTheseON = new GameObject[script.NumberOfThingsToTurnOn];
            }
            for (int i = 0; i < script.NumberOfThingsToTurnOn;i++)
            {
                if (tempOns != null)
                {
                    if (tempOns.Length > i)
                    {
                        script.ClickTurnsTheseON[i] = tempOns[i];
                    }
                }
                script.ClickTurnsTheseON[i] = (GameObject)EditorGUILayout.ObjectField("Object to Turn On", script.ClickTurnsTheseON[i], typeof(GameObject), true);
            }

            script.NumberOfThingsToTurnOff = EditorGUILayout.IntField("Number of Things to Turn Off", script.NumberOfThingsToTurnOff);
            GameObject[] tempOffs = script.ClickTurnsTheseOFF;
            if (previousOffCount != script.NumberOfThingsToTurnOff)
            {
                script.ClickTurnsTheseOFF = new GameObject[script.NumberOfThingsToTurnOff];
            }
            for (int i = 0; i < script.NumberOfThingsToTurnOff; i++)
            {
                if (tempOffs != null)
                {
                    if (tempOffs.Length > i)
                    {
                        script.ClickTurnsTheseOFF[i] = tempOffs[i];
                    }
                }

                script.ClickTurnsTheseOFF[i] = (GameObject)EditorGUILayout.ObjectField("Object to Turn Off", script.ClickTurnsTheseOFF[i], typeof(GameObject), true);
            }
            previousOnCount = script.NumberOfThingsToTurnOn;
            previousOffCount = script.NumberOfThingsToTurnOff;
        }
        else if (script.myType == InteractionController.InteractionType.MakeCameraChild)
        {
            //make camera child thing
            script.objectToBecomeParent = (GameObject)EditorGUILayout.ObjectField("Object to Become Camera Parent", script.objectToBecomeParent, typeof(GameObject), true);
        }
    }
}