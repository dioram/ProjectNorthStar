/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using UnityEditor;
using UnityEngine;

namespace Leap.Unity
{
    [CustomEditor(typeof(LeapImageRetriever))]
    public class LeapImageRetrieverEditor : CustomEditorBase<LeapImageRetriever>
    {

        private GUIContent _textureGUIContent;
        private GUIContent _distortionTextureGUIContent;

        protected override void OnEnable()
        {
            base.OnEnable();

            _textureGUIContent = new GUIContent("Sensor Texture");
            _distortionTextureGUIContent = new GUIContent("Distortion Texture");
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            if (Application.isPlaying)
            {
                var data = target.TextureData;
                var dataType = typeof(Object);

                EditorGUI.BeginDisabledGroup(true);
                for (int i = 0; i < data.TextureData.Count; i++)
                {
                    EditorGUILayout.ObjectField(_textureGUIContent, data.TextureData[i].CombinedTexture, dataType, true);
                }
                EditorGUILayout.ObjectField(_distortionTextureGUIContent, data.Distortion.CombinedTexture, dataType, true);
                EditorGUI.EndDisabledGroup();
            }
        }
    }
}
