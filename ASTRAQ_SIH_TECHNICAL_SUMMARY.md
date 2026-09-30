# ASTRAQ x SIH 26169 - MASTER TECHNICAL REPORT (Part 1 of 2)
# Sections 1-12: Forensic Reconstruction + PS Compliance Analysis

## 1. EXECUTIVE SUMMARY

ASTRAQ is a browser-first, dual-engine simulation platform for coarse Pointing, Acquisition and Tracking (PAT) of a mobile FSOC terminal.

After forensic inspection of engine.ts (711 lines), centroid.ts (444 lines), kalman.ts, pid.ts, gimbal.ts, supervisor.ts, disturbance.ts, sensor.ts, trajectory.ts, video/analyzer.ts, analysis/metrics.ts, server/astraq_engine/engine.py (557 lines), plus FORENSIC_AUDIT.md and all test results, the following is confirmed:

ASTRAQ tracking loop is technically sound and significantly more complete than a typical SIH submission. Core pipeline: CentroidDetector -> MLP verifier -> AngularKalman -> PID + feed-forward -> first-order Gimbal -> 7-state Supervisor.

PS compliance PASS=35, PARTIAL=5, MISSING=1 out of 41 requirements.

## 2. KEY ASTRAQ STRENGTHS

1. CentroidDetector: 7-step O(N) pipeline, sub-pixel accuracy <0.5 px nominal, <1.5 px at PS max
2. MLP Verifier: 11->16->8->1, AUC=0.9997 (simulated), FA 8.4% -> 0.59%, <0.1ms inference
3. AngularKalman: world coordinates (not pixels), latency compensation, adaptive R, cosine correction
4. PID+FF: anti-windup, integral separation, Kalman velocity feed-forward (4x RMS improvement)
5. Gimbal: first-order rate servo, rate+accel limits, wind disturbance at correct location
6. Supervisor FSM: 7-state, observable-only transitions, M-of-N confirmation
7. SpiralSearch: 85% overlap, reacq spiral centered on KF prediction
8. 9-preset Monte Carlo: deterministic (mulberry32), 20 seeds x 9 scenarios
9. Video benchmark path: correctly bypasses PTZ camera (PS Benchmark-2)
10. Dual TS+Python engine parity

## 3. VERIFIED LIMITATIONS

L1 (P1): Acquisition time 6.67s mean with 4-only FOV (physical constraint, not bug)
L2 (P1): High-jitter (+-20px) -> 17.44px RMS (physical floor)
L3 (P1): Video benchmark err metric = center-distance without truth CSV
L4 (P1): No adaptive decision layer above Supervisor
L5 (P2): Verifier trained on simulated data only
L6 (P2): YOLO placeholder without implementation
L7 (P2): Screen size 2000x2000 interpretation ambiguity
L8 (P2): SyntheticDetector uses ground truth (labeled, expected)
L9 (P3): No demo video
L10 (P3): PixelKalman duplicates AxisFilter code

## 4. PS COMPLIANCE MATRIX (Summary)

PASS (35): virtual environment, beacon, pan-tilt camera, detection, tracking, camera control,
  camera type/resolution/FOV/rate, all 7 trajectories, pan/tilt speed, control rate,
  target loss, re-acquisition, processing speed, all noise types, all atmospheres,
  platform motion, real-time stats, performance log, video benchmark MP4 input,
  standalone executable, documentation, technical report, user manual

PARTIAL (5):
  - Screen size: scene is 2000x2000 but sensor is 640x480 (interpretation ambiguity)
  - Acquisition time: passes with wide-field (1.13s), fails with 4-only (6.67s)
  - Tracking error: most scenarios <=5px, high-jitter 17.44px
  - Video centroid error: correct with truth CSV, center-distance without
  
MISSING (1): Demo video 3-5 min

## 5. OUR ENGINEERING CONTRIBUTION: ADAPTIVE TRACKING DECISION LAYER (ATDL)

The Supervisor makes binary state transitions. We add a continuous quality monitoring layer:

Tracking Health Index (THI):
  THI = 0.25*S_conf + 0.25*S_uncertainty + 0.25*S_miss + 0.25*S_error_trend
  All S_i in [0,1] where 1=good, 0=bad

ATDL Modes:
  NORMAL (THI > 0.8): no change
  PREDICTIVE (0.5 < THI <= 0.8): widen gate, increase FF weight
  PRE-LOSS (0.2 < THI <= 0.5): wider ROI, larger gate multiplier
  RECOVERY (THI <= 0.2 or Supervisor->LOST): reacq spiral

Implementation:
  - New module: core/tracking/adaptive.ts (~120 lines)
  - Mirror: server/astraq_engine/adaptive.py
  - Integration: engine.ts:controlLaw() reads ATDL output (gateMult, kpScale, mode)
  - Metrics: modeHistory and modeDistribution added to RunMetrics
  - THI gauge added to HUD
  - ATDL has enabled flag: disabled = original ASTRAQ behavior exactly

## 6. COMPONENT DECISION MATRIX

KEEP (16): CentroidDetector, MLP verifier, DetectionProvider interface, AngularKalman,
  PID+FF, Gimbal, Supervisor FSM, SpiralSearch, Disturbance model, 9-preset benchmark,
  Video benchmark path, Dual TS+Python parity, Config system, Report generation,
  Batch worker, Demo mode

MODIFY (4):
  - PID: accept ATDL kpScale multiplier
  - Metrics: add modeHistory and modeDistribution
  - Report: add ATDL mode distribution section
  - Demo script: add ATDL phase showcase

ADD (3):
  - adaptive.ts (ATDL TypeScript module)
  - adaptive.py (Python mirror)
  - Demo video (PS deliverable, not yet recorded)

REMOVE (0): Nothing removed

## 7. IMPLEMENTATION PLAN

Phase 0 (0.5 days): Freeze baseline, run all 67+36 tests, record metrics
Phase 1 (1 day): PS compliance fixes, video metric labeling, electron build verify
Phase 2 (1.5 days): ATDL module, engine integration, THI HUD display
Phase 3 (0.5 days): Recovery/reacquisition validation under ATDL
Phase 4 (0.5 days): ATDL metrics in RunMetrics and report
Phase 5 (0.5 days): Full validation, ablation ATDL on/off comparison
Phase 6 (1 day): Demo video recording, screenshots, PPT

Total: ~4.5 developer-days

## 8. NOVELTY CLAIMS

Safe to claim:
  - ATDL provides continuous graded quality monitoring above binary state machine
  - THI drives proactive mode selection before track loss
  - Mode distribution reporting quantifies adaptive behavior
  - Integration within PS-compliant FSOC simulation context

Do NOT claim:
  - Global novelty of adaptive tracking (well-established)
  - MLP verifier is novel (it is existing ASTRAQ)
  - Real-world AI generalization (simulated data only)
  - YOLO was considered and rejected for technical reasons (not implemented at all)

## 9. SIH RISKS

HIGH: Acquisition time question (4-only FOV) -> prepared explanation (physical constraint)
MEDIUM: Screen size 2000x2000 question -> prepared explanation (scene vs sensor)
MEDIUM: Verifier AUC claim -> explicitly label as simulated data performance
LOW: YOLO question -> blob detection + MLP is correct tool for 5-20px spots
LOW: Demo video not recorded -> schedule Phase 6

## 10. FINAL ARCHITECTURE SUMMARY

PATH A (Simulation):
  Beacon -> Disturbances -> SensorRenderer -> CentroidDetector+MLP ->
  AngularKalman -> [ATDL NEW] -> PID+FF -> Gimbal -> Feedback

PATH B (External MP4, PS Benchmark-2):
  MP4 Frame -> rgbaToGray -> CentroidDetector+MLP ->
  PixelKalman -> state: SEARCHING|TRACKING -> VideoReport

Both paths converge at CentroidDetector. Path A has gimbal; Path B does not.
ATDL applies to Path A only (gimbal state required for mode decisions).
