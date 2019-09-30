using UnityEngine;
public class CalibrationWindowOffset : MonoBehaviour
{
    void Start()
    {
        Screen.fullScreenMode = FullScreenMode.FullScreenWindow;
        Screen.fullScreen = true;
        if (Display.displays.Length == 4)
        {
            for (int i = 0; i < 4; i++)
            {
                if (!Display.displays[i].active)
                {
                    Display.displays[i].Activate();
                }
            }
        }
    }
}
