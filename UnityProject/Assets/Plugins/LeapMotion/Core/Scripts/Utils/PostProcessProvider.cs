/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using Leap.Unity.Attributes;
using UnityEngine;

namespace Leap.Unity
{

    public abstract class PostProcessProvider : LeapProvider
    {

        [Tooltip("The LeapProvider whose output hand data will be copied, modified, and "
               + "output by this post-processing provider.")]
        [SerializeField]
        [OnEditorChange("inputLeapProvider")]
        protected LeapProvider _inputLeapProvider;
        public LeapProvider inputLeapProvider
        {
            get { return _inputLeapProvider; }
            set
            {
                if (Application.isPlaying && _inputLeapProvider != null)
                {
                    _inputLeapProvider.OnFixedFrame -= processFixedFrame;
                    _inputLeapProvider.OnUpdateFrame -= processUpdateFrame;
                }

                _inputLeapProvider = value;
                validateInput();

                if (Application.isPlaying && _inputLeapProvider != null)
                {
                    _inputLeapProvider.OnFixedFrame -= processFixedFrame; // safeguard double-subscription
                    _inputLeapProvider.OnFixedFrame += processFixedFrame;
                    _inputLeapProvider.OnUpdateFrame -= processUpdateFrame; // safeguard double-subscription
                    _inputLeapProvider.OnUpdateFrame += processUpdateFrame;
                }
            }
        }

        public enum DataUpdateMode { UpdateOnly, FixedUpdateOnly, UpdateAndFixedUpdate }
        [Tooltip("Whether this post-processing provider should process data received from "
               + "Update frames, FixedUpdate frames, or both. Processing both kinds of frames "
               + "is only recommended if your post-process is stateless.")]
        public DataUpdateMode dataUpdateMode = DataUpdateMode.UpdateOnly;

        [Tooltip("When this setting is enabled, frame data is passed from this provider's "
               + "input directly to its output without performing any post-processing.")]
        public bool passthroughOnly = false;

        private Frame _cachedUpdateFrame = new Frame();
        private Frame _cachedFixedFrame = new Frame();

        public override Frame CurrentFrame
        {
            get
            {
#if UNITY_EDITOR
                if (!Application.isPlaying && _inputLeapProvider != null)
                {
                    processUpdateFrame(_inputLeapProvider.CurrentFrame);
                }
#endif
                return _cachedUpdateFrame;
            }
        }

        public override Frame CurrentFixedFrame
        {
            get
            {
#if UNITY_EDITOR
                if (!Application.isPlaying && _inputLeapProvider != null)
                {
                    processUpdateFrame(_inputLeapProvider.CurrentFixedFrame);
                }
#endif
                return _cachedFixedFrame;
            }
        }

        protected virtual void OnEnable()
        {
            // Bootstrap event subscription, handled in the input property setter.
            inputLeapProvider = _inputLeapProvider;
        }

        protected virtual void OnValidate()
        {
            validateInput();
        }

        public abstract void ProcessFrame(ref Frame inputFrame);

        private void validateInput()
        {
            if (detectCycle())
            {
                _inputLeapProvider = null;
                Debug.LogError("The input to the post-process provider on " + gameObject.name
                             + " causes an infinite cycle, so its input has been set to null.");
            }
        }

        private bool detectCycle()
        {
            LeapProvider providerA = _inputLeapProvider, providerB = _inputLeapProvider;
            while (providerA is PostProcessProvider)
            {
                providerB = (providerB as PostProcessProvider).inputLeapProvider;
                if (providerA == providerB) { return true; }
                else if (!(providerB is PostProcessProvider)) { return false; }
                providerA = (providerA as PostProcessProvider).inputLeapProvider;
                providerB = (providerB as PostProcessProvider).inputLeapProvider;
                if (!(providerB is PostProcessProvider)) { return false; }
            }
            return false;
        }

        private void processUpdateFrame(Frame inputFrame)
        {
            if (dataUpdateMode == DataUpdateMode.FixedUpdateOnly)
            {
                return;
            }

            _cachedUpdateFrame.CopyFrom(inputFrame);
            if (!passthroughOnly) { ProcessFrame(ref _cachedUpdateFrame); }
            DispatchUpdateFrameEvent(_cachedUpdateFrame);
        }

        private void processFixedFrame(Frame inputFrame)
        {
            if (dataUpdateMode == DataUpdateMode.UpdateOnly)
            {
                return;
            }

            _cachedFixedFrame.CopyFrom(inputFrame);
            if (!passthroughOnly) { ProcessFrame(ref _cachedFixedFrame); }
            DispatchFixedFrameEvent(_cachedFixedFrame);
        }

    }
}
