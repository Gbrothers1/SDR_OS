import React, { createContext, useContext, useState, useEffect, useRef, useCallback } from 'react';
import { useGenesis } from './GenesisContext';

const PhaseContext = createContext();

export const usePhase = () => {
  const context = useContext(PhaseContext);
  if (!context) {
    throw new Error('usePhase must be used within a PhaseProvider');
  }
  return context;
};

// Compute trend direction from a rolling window of values
const computeTrend = (history, windowSize = 10) => {
  if (history.length < 2) return { direction: 'flat', unstable: false };

  const recent = history.slice(-windowSize);
  const first = recent[0];
  const last = recent[recent.length - 1];
  const delta = last - first;

  // Compute variance
  const mean = recent.reduce((a, b) => a + b, 0) / recent.length;
  const variance = recent.reduce((a, b) => a + (b - mean) ** 2, 0) / recent.length;
  const coeffOfVariation = mean !== 0 ? Math.sqrt(variance) / Math.abs(mean) : 0;

  const threshold = Math.abs(mean) * 0.05; // 5% change counts as a trend
  let direction = 'flat';
  if (delta > threshold) direction = 'rising';
  else if (delta < -threshold) direction = 'falling';

  return {
    direction,
    unstable: coeffOfVariation > 0.3, // >30% CoV = unstable
  };
};

export const PhaseProvider = ({ children, ros, rosConnected }) => {
  const {
    genesisConnected,
    bridgeConnected,
    trainingMetrics,
    genesisMode,
    blendAlpha,
    deadmanActive,
    safetyFlags,
    actorTag,
    isPaused,
    scriptStatus,
    scriptError,
    currentFrame,
    frameStats,
    commandSource,
    videoHealthy,
  } = useGenesis();

  // Reward history for trend computation
  const rewardHistoryRef = useRef([]);
  const [rewardTrend, setRewardTrend] = useState({ direction: 'flat', unstable: false });

  // Last-message timestamps for health pulse
  const rosLastMsgRef = useRef(0);
  const genesisLastFrameRef = useRef(0);
  const trainLastMetricRef = useRef(0);

  const [rosHealth, setRosHealth] = useState('dead');
  const [genesisHealth, setGenesisHealth] = useState('dead');
  const [trainHealth, setTrainHealth] = useState('dead');

  // Track ROS message timestamps
  useEffect(() => {
    if (rosConnected) {
      rosLastMsgRef.current = Date.now();
      setRosHealth('ok');
    } else {
      setRosHealth('dead');
    }
  }, [rosConnected]);

  // Track Genesis frame timestamps (covers both JPEG and H.264)
  useEffect(() => {
    if (currentFrame) {
      genesisLastFrameRef.current = Date.now();
    }
    if (videoHealthy && genesisConnected) {
      setGenesisHealth('ok');
    }
  }, [currentFrame, videoHealthy, genesisConnected]);

  // Track training metrics timestamps + reward history
  useEffect(() => {
    if (trainingMetrics) {
      trainLastMetricRef.current = Date.now();
      setTrainHealth('ok');

      if (trainingMetrics.total_reward != null) {
        rewardHistoryRef.current = [
          ...rewardHistoryRef.current.slice(-99),
          trainingMetrics.total_reward,
        ];
        setRewardTrend(computeTrend(rewardHistoryRef.current));
      }
    }
  }, [trainingMetrics]);

  // Health degradation check (run every second)
  useEffect(() => {
    const interval = setInterval(() => {
      const now = Date.now();

      // ROS health
      if (!rosConnected) {
        setRosHealth('dead');
      } else if (now - rosLastMsgRef.current > 5000) {
        setRosHealth('degraded');
      } else {
        setRosHealth('ok');
      }

      // Genesis health — videoHealthy covers both JPEG and H.264 frames
      if (!genesisConnected && !bridgeConnected) {
        setGenesisHealth('dead');
      } else if (genesisConnected && !videoHealthy) {
        setGenesisHealth('degraded');
      } else if (genesisConnected) {
        setGenesisHealth('ok');
      }

      // Training health
      const isTraining = scriptStatus === 'running' &&
        (genesisMode === 'hil_blend' || genesisMode === 'online_finetune');
      if (!isTraining) {
        setTrainHealth(trainingMetrics ? 'ok' : 'dead');
      } else if (now - trainLastMetricRef.current > 10000) {
        setTrainHealth('degraded');
      }
    }, 1000);

    return () => clearInterval(interval);
  }, [rosConnected, genesisConnected, bridgeConnected, videoHealthy, scriptStatus, genesisMode, trainingMetrics]);

  // Compute authority
  let authority = 'human';
  if (blendAlpha >= 0.95) authority = 'policy';
  else if (blendAlpha > 0.05) authority = 'blend';

  // Compute active phase
  let activePhase = 'idle';
  if (genesisMode === 'eval') {
    activePhase = 'eval';
  } else if (scriptStatus === 'running' &&
    (genesisMode === 'hil_blend' || genesisMode === 'online_finetune')) {
    activePhase = 'train';
  } else if (rosConnected || genesisConnected) {
    activePhase = 'teleop';
  }

  // Compute safety clamp count from training metrics
  const safetyClampCount = trainingMetrics?.safety_clamp_count ?? 0;
  const safetyClampActive = Object.values(safetyFlags).some(Boolean);

  // Eval risk level
  let evalRisk = 'nominal';
  if (safetyClampActive && safetyClampCount >= 3) evalRisk = 'anomaly';
  else if (safetyClampActive) evalRisk = 'clamped';

  // Is recording?
  const isRecording = genesisMode === 'teleop_record' && scriptStatus === 'running';
  const recordingStep = trainingMetrics?.step_count ?? null;
  const recordingTotal = trainingMetrics?.total_steps ?? null;

  // Training state
  const trainEpoch = trainingMetrics?.epoch ?? null;
  const trainTotalEpochs = trainingMetrics?.total_epochs ?? null;
  const trainReward = trainingMetrics?.total_reward ?? null;

  // Error surfacing: pick the most relevant error
  let stageError = null;
  if (scriptError) {
    stageError = scriptError;
  }

  const value = {
    activePhase,
    authority,
    blendAlpha,

    stages: {
      teleop: {
        health: rosConnected ? rosHealth : (genesisConnected ? genesisHealth : 'dead'),
        actor: actorTag,
        commandSource: commandSource || 'gamepad',
        deadmanActive,
        recording: isRecording,
        recordingStep,
        recordingTotal,
      },
      train: {
        health: trainHealth,
        trend: rewardTrend.direction,
        unstable: rewardTrend.unstable,
        reward: trainReward,
        epoch: trainEpoch,
        totalEpochs: trainTotalEpochs,
        paused: isPaused,
        error: scriptStatus === 'error' ? scriptError : null,
      },
      eval: {
        health: genesisMode === 'eval' ? genesisHealth : 'dead',
        riskLevel: evalRisk,
        clampCount: safetyClampCount,
        safetyFlags,
        error: genesisMode === 'eval' && scriptError ? scriptError : null,
      },
    },

    // Connection state (for ViewerLayer source selection)
    rosConnected: !!rosConnected,
    genesisConnected,
    bridgeConnected,
  };

  return (
    <PhaseContext.Provider value={value}>
      {children}
    </PhaseContext.Provider>
  );
};
