/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using UnityEngine;
using UnityEngine.Events;

namespace Leap.Unity.Examples
{

    [AddComponentMenu("")]
    public class SimpleFacingCameraCallbacks : MonoBehaviour
    {

        public Transform toFaceCamera;
        public Camera cameraToFace;

        private bool _initialized = false;
        private bool _isFacingCamera = false;

        public UnityEvent OnBeginFacingCamera;
        public UnityEvent OnEndFacingCamera;

        void Start()
        {
            if (toFaceCamera != null) initialize();
        }

        private void initialize()
        {
            if (cameraToFace == null) { cameraToFace = Camera.main; }
            // Set "_isFacingCamera" to be whatever the current state ISN'T, so that we are
            // guaranteed to fire a UnityEvent on the first initialized Update().
            _isFacingCamera = !GetIsFacingCamera(toFaceCamera, cameraToFace);
            _initialized = true;
        }

        void Update()
        {
            if (toFaceCamera != null && !_initialized)
            {
                initialize();
            }
            if (!_initialized) return;

            if (GetIsFacingCamera(toFaceCamera, cameraToFace, _isFacingCamera ? 0.77F : 0.82F) != _isFacingCamera)
            {
                _isFacingCamera = !_isFacingCamera;

                if (_isFacingCamera)
                {
                    OnBeginFacingCamera.Invoke();
                }
                else
                {
                    OnEndFacingCamera.Invoke();
                }
            }
        }

        public static bool GetIsFacingCamera(Transform facingTransform, Camera camera, float minAllowedDotProduct = 0.8F)
        {
            return Vector3.Dot((camera.transform.position - facingTransform.position).normalized, facingTransform.forward) > minAllowedDotProduct;
        }

    }

}
