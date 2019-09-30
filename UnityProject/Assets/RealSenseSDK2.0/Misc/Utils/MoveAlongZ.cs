using UnityEngine;

public class MoveAlongZ : MonoBehaviour
{

    public float PositionZ
    {
        set
        {
            transform.position = Vector3.forward * value;
        }
    }
}
