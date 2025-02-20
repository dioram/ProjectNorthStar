/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using UnityEngine;

namespace Leap.Unity.Interaction
{

    #region IInteractionBehaviour

    public static class IInteractionBehaviourExtensions
    {

        public static bool ShouldIgnoreHover(this IInteractionBehaviour intObj, InteractionController controller)
        {
            switch (intObj.ignoreHoverMode)
            {
                case IgnoreHoverMode.None: return false;
                case IgnoreHoverMode.Left: return !controller.isTracked || controller.isLeft;
                case IgnoreHoverMode.Right: return !controller.isTracked || controller.isRight;
                case IgnoreHoverMode.Both: default: return true;
            }
        }

    }

    #endregion

    #region Vector3

    public static class Vector3Extensions
    {

        public static Vector3 ConstrainToSegment(this Vector3 position, Vector3 a, Vector3 b)
        {
            Vector3 ba = b - a;
            return Vector3.Lerp(a, b, Vector3.Dot(position - a, ba) / ba.sqrMagnitude);
        }

        public static float LargestComp(this Vector3 v)
        {
            return Mathf.Max(Mathf.Max(v.x, v.y), v.z);
        }

        public static int LargestCompIdx(this Vector3 v)
        {
            return v.x > v.y ?
                     v.x > v.z ?
                       0 // x > y, x > z
                     : 2 // x > y, z > x
                   : v.y > v.z ?
                     1   // y > x, y > z
                   : 2;  // y > x, z > y
        }

        public static Vector3 Abs(this Vector3 v)
        {
            return new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
        }

    }

    #endregion

}
