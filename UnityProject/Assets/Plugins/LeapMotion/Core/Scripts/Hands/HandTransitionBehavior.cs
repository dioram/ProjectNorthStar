/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using UnityEngine;

namespace Leap.Unity
{

    /// <summary>
    /// A component to be attached to a HandModelBase to handle starting and ending of
    /// tracking.
    /// </summary>
    public abstract class HandTransitionBehavior : MonoBehaviour
    {

        protected HandModelBase handModelBase;

        protected abstract void HandReset();
        protected abstract void HandFinish();

        protected virtual void Awake()
        {
            handModelBase = GetComponent<HandModelBase>();
            if (handModelBase == null)
            {
                Debug.LogWarning("HandTransitionBehavior components require a HandModelBase "
                  + "component attached to the same GameObject. (Awake)");
                return;
            }

            handModelBase.OnBegin -= HandReset;
            handModelBase.OnBegin += HandReset;

            handModelBase.OnFinish -= HandFinish;
            handModelBase.OnFinish += HandFinish;
        }

        protected virtual void OnDestroy()
        {
            if (handModelBase == null)
            {
                HandModelBase handModelBase = GetComponent<HandModelBase>();
                if (handModelBase == null)
                {
                    Debug.LogWarning("HandTransitionBehavior components require a HandModelBase "
                      + "component attached to the same GameObject. (OnDestroy)");
                    return;
                }
            }

            handModelBase.OnBegin -= HandReset;
            handModelBase.OnFinish -= HandFinish;
        }
    }
}
