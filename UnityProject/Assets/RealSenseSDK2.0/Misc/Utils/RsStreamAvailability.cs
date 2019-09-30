using UnityEngine;
using UnityEngine.Events;

public class RsStreamAvailability : MonoBehaviour
{
    public RsConfiguration DeviceConfiguration;

    [Space]
    public UnityEvent OnDeviceAvailable;
    public UnityEvent OnDeviceUnAvailable;
}
