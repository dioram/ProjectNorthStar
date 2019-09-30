using System.Runtime.CompilerServices;
using UnityEngine;

namespace Leap.Unity.AR
{
    class DebugEx : Debug
    {
        public static void Log(object message, [CallerMemberName]string callerName = "")
        {
            Debug.Log($"[{callerName}]: {message}");
        }
    }
}
