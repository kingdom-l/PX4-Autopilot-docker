# PX4 eADRC-HRP and SACT+ port

## Controller selection

`HY_DEPVA_PID_EN` remains the selector so existing parameter files keep their
original meaning for values 0 and 1:

| Value | Controller |
|---:|---|
| 0 | Existing PX4 ADRC |
| 1 | Existing PX4 PID (default) |
| 2 | Lightweight eADRC-HRP |
| 3 | SACT+ |

Changing the mode resets the PID integrals, HRP history, eADRC observer, and
SACT+ adaptive states. Do not change controller mode during an unrestricted
in-water run.

## Output contract

The new controllers calculate physical force internally but preserve the
existing module interface:

- `_fx_sp` is mapped from force to normalized forward throttle in `[0, 1]`
  with `2 * HY_THRUST_MAX`, then written to `thrust_body[0]`.
  `HY_VE_RES_ADRC` is used only as the minimum nonzero command. Negative
  force requests map to zero because this allocator does not command reverse
  thrust.
- `_fz_sp` is a signed vertical-force request in newtons, limited by
  `HY_DEP_LIM_ADRC`, then written directly to `thrust_body[2]`.
- `HY_VA_FF_ADRC * HY_VA_SP` and `HY_DEP_FF_ADRC` remain the forward and
  vertical feedforward terms for both new controllers.

This deliberately does not copy MATLAB's final `mass * control` command
directly into `_fx_sp`: it first divides the requested force by
`2 * HY_THRUST_MAX`, exactly reversing the conversion in the hydro allocator.
The eADRC observer is fed the force that the normalized interface actually
requests, including zero-command clipping.

## eADRC-HRP changes for STM32H743

The controller retains the MATLAB error-domain ESOs and five-feature model:

`[1, r(k-1), r(k-2), e(k), estimated error rate]`.

The deployed predictor differs in the following bounded ways:

- fixed memory, with `HY_HR_WIN=40` and a compile-time maximum of 40;
- confidence-gated, exponentially weighted ridge regression;
- standardized features and a fixed 5-by-5 Cholesky solve;
- coefficients refitted once per `HY_HR_DECIM` samples (default 5) and held
  between fits;
- no heap allocation, Eigen matrix, pseudoinverse, or full condition-number
  decomposition;
- ESO Euler substeps no larger than 10 ms for variable-rate stability;
- the depth residual uses the measured depth-error rate, avoiding a direct
  second difference of noisy depth measurements.

The two predictor buffers and controller state occupy about 2.2 kB with the
current 40-sample compile-time limit.

### eADRC-HRP parameters

| MATLAB parameter | PX4 parameter | Default |
|---|---|---:|
| `dep_kp_start` | `HY_HR_D_KP` | 190 |
| `dep_kd_start` | `HY_HR_D_KD` | 100 |
| `dep_wo_start` | `HY_HR_D_WO` | 20 rad/s |
| `vel_kp_start` | `HY_HR_V_KP` | 4.2 |
| `vel_wo_start` | `HY_HR_V_WO` | 8 rad/s |
| depth prediction gain | `HY_HR_D_ALP` | 1 |
| velocity prediction gain | `HY_HR_V_ALP` | 1 |
| `M` | `HY_HR_WIN` | 40 |
| `min_samples` | `HY_HR_MIN` | 30 |
| fit interval | `HY_HR_DECIM` | 5 samples |
| `forget_factor` | `HY_HR_FORGET` | 0.99 |
| `ridge_lambda` | `HY_HR_RIDGE` | 0.01 |
| `residual_lpf` | `HY_HR_RLPF` | 0.95 at 100 Hz |
| `r_limit_dep` | `HY_HR_D_RLIM` | 8 m/s^2 |
| `r_limit_vel` | `HY_HR_V_RLIM` | 4 m/s^2 |
| observer compensation ramp | `HY_HR_RAMP` | 0.15 s |

There are no `*_start`/`*_run` pairs and no switching time. The listed gains
are the MATLAB startup values used continuously.

## SACT+ deployment changes

SACT+ likewise uses only the MATLAB startup gains. `sp_run`, `sd_run`, and
`switch_time` are not present. The derivative filter is represented by a
time constant (`HY_SA_DER_TC=0.027 s`), approximately equivalent to the
MATLAB coefficient 0.73 at 100 Hz but robust to variable `dt`.

`HY_SA_D_LLM` and `HY_SA_V_LLM` bound the two Lambda states. These are
deployment safety limits added to prevent adaptive-state windup while the
actuator output is saturated.

### SACT+ channel parameters

For each channel, replace `D` below with `D` for depth or `V` for velocity:

| Meaning | PX4 suffix | Depth default | Velocity default |
|---|---|---:|---:|
| proportional scale | `HY_SA_D_SP` / `HY_SA_V_SP` | 43 | 40 |
| derivative scale | `HY_SA_D_SD` / `HY_SA_V_SD` | 45 | 5 |
| proportional boundary | `HY_SA_D_DP` / `HY_SA_V_DP` | 0.065 | 1.0 |
| derivative boundary | `HY_SA_D_DD` / `HY_SA_V_DD` | 0.50 | 2.0 |
| proportional exponent | `HY_SA_D_DLP` / `HY_SA_V_DLP` | 0.25 | 0.80 |
| derivative exponent | `HY_SA_D_DLD` / `HY_SA_V_DLD` | 0.50 | 0.80 |
| `alpha1` | `HY_SA_D_A1` / `HY_SA_V_A1` | 0.10 | 0.10 |
| `alpha2` | `HY_SA_D_A2` / `HY_SA_V_A2` | 0.10 | 0.05 |
| `gamma` | `HY_SA_D_GAM` / `HY_SA_V_GAM` | 1.0 | 0.50 |
| `Dnom` | `HY_SA_D_DNM` / `HY_SA_V_DNM` | 16 | 3 |
| theta limit | `HY_SA_D_TLM` / `HY_SA_V_TLM` | 20 | 10 |
| Lambda limit | `HY_SA_D_LLM` / `HY_SA_V_LLM` | 20 | 10 |

The common compensation ramp is `HY_SA_RAMP=1.4 s`.

## Recommended bench-to-water tuning order

1. Verify signs and output units with propellers disabled. Confirm
   `thrust_body[0]` remains in `[0,1]` and `thrust_body[2]` remains inside
   `+-HY_DEP_LIM_ADRC`.
2. Set `HY_HR_D_ALP=0` and `HY_HR_V_ALP=0`. Tune the eADRC base loop first:
   `B0_INV`, feedforward, `KP/KD`, then observer bandwidth.
3. Start HRP with `HY_HR_WIN=30`, `HY_HR_MIN=30`, `HY_HR_DECIM=5`, and raise
   each alpha from 0 in steps of 0.2. Move to a 40-sample window only after
   checking the disturbance-onset spectrum and actuator command.
4. For SACT+, tune the proportional/derivative scales with `A1`, `A2`, and
   `GAM` set to zero. Then enable the adaptive terms gradually and check both
   Lambda limits during command saturation.
5. Only increase ESO bandwidth or reduce derivative filtering after checking
   depth/velocity noise and the actual controller update rate in a ULog.

The existing `vehicle_local_position_setpoint` debug publication reports the
active mode in `yawspeed`. In eADRC-HRP mode it also reports observer states
and the depth residual prediction; in SACT+ mode it reports Lambda/theta
states.
