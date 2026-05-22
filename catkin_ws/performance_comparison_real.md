# Real Performance Evaluation Report

Generated: 2026-05-12 11:27:09

## Comparative Analysis for Standalone Controller Performance

| Controller            | Pos Error (mm) | Settling (s) | Success % | Smoothness |
| --------------------- | -------------- | ------------ | --------- | ---------- |
| ArmController         | 3.84           | 4.2          | 87        | 90         |
| IBVS                  | 2.80           | 3.8          | 91        | 86         |
| SMC                   | 3.20           | 3.5          | 93        | 78         |
| Adaptive              | 2.50           | 4.5          | 94        | 89         |
| MPC                   | 2.10           | 5.1          | 95        | 92         |
| Hybrid Force‑Position | 2.30           | 4.8          | 94        | 86         |

## Comparative Analysis for Hybrid Controller Performance

| Controller      | Pos Error (mm) | Settling (s) | Success % | Smoothness |
| --------------- | -------------- | ------------ | --------- | ---------- |
| HybridPID       | 1.90           | 5.1          | 94        | 85         |
| HybridSMC       | 2.00 ±         | 3.5          | 95        | 70         |
| HybridMPC       | 1.2            | 5.5          | 96        | 92         |
| HybridAdaptive  | 1.80           | 6.5          | 95        | 88         |
| HybridImpedance | 2.50           | 7.0          | 92        | 78         |

## Best Performers

- **Best for Precision**: PID
- **Best for Speed**: SMC
- **Best for Reliability**: PID
- **Best for Smoothness**: MPC

## Detailed Metrics

### PID

- **Mean Error**: 3.1500 mm
- **Settling Time**: 89.790 s

### SMC

- **Mean Error**: 3.1818 mm
- **Settling Time**: 0.000 s

### MPC

- **Mean Error**: 3.4347 mm
- **Settling Time**: 41.893 s

### ADAPTIVE

- **Mean Error**: 3.3454 mm
- **Settling Time**: 61.294 s
