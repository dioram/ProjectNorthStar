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
using UnityEngine.Serialization;

namespace Leap.Unity.GraphicalRenderer
{

    public static class LeapSpriteFeatureExtension
    {
        public static LeapSpriteData Sprite(this LeapGraphic graphic)
        {
            return graphic.GetFeatureData<LeapSpriteData>();
        }
    }

    [LeapGraphicTag("Sprite")]
    [Serializable]
    public class LeapSpriteData : LeapFeatureData
    {

        [FormerlySerializedAs("sprite")]
        [EditTimeOnly, SerializeField]
        private Sprite _sprite;

        public Sprite sprite
        {
            get
            {
                return _sprite;
            }
            set
            {
                _sprite = value;
                graphic.isRepresentationDirty = true;
                MarkFeatureDirty();
            }
        }
    }
}
