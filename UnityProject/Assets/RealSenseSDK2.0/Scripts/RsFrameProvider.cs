﻿using Intel.RealSense;
using System;
using UnityEngine;

public abstract class RsFrameProvider : MonoBehaviour
{
    public bool Streaming { get; protected set; }
    public PipelineProfile ActiveProfile { get; protected set; }

    public abstract event Action<PipelineProfile> OnStart;
    public abstract event Action OnStop;
    public abstract event Action<Frame> OnNewSample;
}

