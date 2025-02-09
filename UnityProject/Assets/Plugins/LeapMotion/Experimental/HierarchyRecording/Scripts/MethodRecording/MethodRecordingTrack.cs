/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using UnityEngine.Timeline;

namespace Leap.Unity.Recording
{

    [TrackColor(0.1f, 0.1f, 1.0f)]
    [TrackClipType(typeof(MethodRecordingClip))]
    [TrackBindingType(typeof(MethodRecording))]
    public class MethodRecordingTrack : TrackAsset { }
}
