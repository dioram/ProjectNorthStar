/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using Leap.Unity.Attributes;
using System;
using UnityEngine;

namespace Leap.Unity.Recording
{

    public class PropertyCompression : MonoBehaviour
    {

        public NamedCompression[] compressionOverrides;

        [Serializable]
        public class NamedCompression
        {
            public string propertyName;

            [MinValue(0)]
            public float maxError;
        }
    }

}
