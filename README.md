# sim2real

Chinese version: [README_zh.md](./README_zh.md)

Full documentation: [https://egalahad.github.io/sim2real/](https://egalahad.github.io/sim2real/)

If you're looking for the HDMI deployment stack, go to [hdmi tag](https://github.com/EGalahad/sim2real/tree/hdmi).

## Quick Start

```bash
# uv sync
```

Run offline motion tracking (sim2sim):

```bash
 conda activate teleopp
 python sim2real/sim_env/base_sim.py --robot g1
 python sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy_config checkpoints/lafan-aa/policy-ec592bb4_lafan_100style_student-5000.yaml
```

After both processes are up, press `]` in the policy terminal to start, then press `9` in the MuJoCo viewer to disable the virtual gantry.

## Next Steps

- [Docs Home](./docs/README.md)
- [Getting Started](./docs/getting-started/README.md)
- [Offline Motion Tracking Tutorial](./docs/tutorials/offline-motion-tracking.md)
- [Pico Teleoperation Tutorial](./docs/tutorials/pico-teleoperation.md)
- [Motion Recording Tutorial](./docs/tutorials/motion-recording.md)


[loop_rate_limiters] [17:38:41] [warning] rate limiter is late by 5.8 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=25.506 ms, budget=20.000 ms, poll=0.065 ms, body_pose=2.855 ms, transform=7.176 ms, retarget=7.562 ms, skip_map=0.000 ms, min_height=0.063 ms, build_payload=0.033 ms, send=3.641 ms
[loop_rate_limiters] [17:38:41] [warning] rate limiter is late by 5.6 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=26.304 ms, budget=20.000 ms, poll=0.081 ms, body_pose=8.256 ms, transform=6.279 ms, retarget=8.013 ms, skip_map=0.000 ms, min_height=0.062 ms, build_payload=0.050 ms, send=3.312 ms
[loop_rate_limiters] [17:38:41] [warning] rate limiter is late by 6.3 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=24.629 ms, budget=20.000 ms, poll=0.057 ms, body_pose=5.956 ms, transform=6.261 ms, retarget=11.738 ms, skip_map=0.000 ms, min_height=0.058 ms, build_payload=0.033 ms, send=0.289 ms
[loop_rate_limiters] [17:38:41] [warning] rate limiter is late by 4.7 [ms] (rate_limiter.py:96)
[Info] mode=live, press x to pause, retargeting avg stats: _body_pose_dict_from_streamer=2.575 ms, self.retarget.retarget=4.146 ms
[Info] mode=live, press x to pause, retargeting avg stats: _body_pose_dict_from_streamer=2.569 ms, self.retarget.retarget=4.137 ms
[Info] paused toggled to True via PICO X button

###
[loop_rate_limiters] [18:33:58] [warning] rate limiter is late by 7.4 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=24.908 ms, budget=20.000 ms, poll=0.075 ms, body_pose=6.086 ms, transform=6.293 ms, retarget=11.793 ms, skip_map=0.000 ms, min_height=0.064 ms, build_payload=0.049 ms, send=0.288 ms
[loop_rate_limiters] [18:33:58] [warning] rate limiter is late by 5.0 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=24.549 ms, budget=20.000 ms, poll=0.097 ms, body_pose=6.384 ms, transform=6.205 ms, retarget=11.180 ms, skip_map=0.000 ms, min_height=0.067 ms, build_payload=0.032 ms, send=0.310 ms
[loop_rate_limiters] [18:33:58] [warning] rate limiter is late by 4.6 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=23.749 ms, budget=20.000 ms, poll=0.074 ms, body_pose=6.000 ms, transform=9.240 ms, retarget=7.806 ms, skip_map=0.000 ms, min_height=0.061 ms, build_payload=0.031 ms, send=0.284 ms
[loop_rate_limiters] [18:33:58] [warning] rate limiter is late by 3.8 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=28.705 ms, budget=20.000 ms, poll=0.066 ms, body_pose=9.216 ms, transform=6.266 ms, retarget=12.338 ms, skip_map=0.000 ms, min_height=0.072 ms, build_payload=0.085 ms, send=0.356 ms
[loop_rate_limiters] [18:33:58] [warning] rate limiter is late by 8.7 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=24.533 ms, budget=20.000 ms, poll=0.070 ms, body_pose=6.388 ms, transform=6.277 ms, retarget=11.136 ms, skip_map=0.000 ms, min_height=0.065 ms, build_payload=0.036 ms, send=0.294 ms
[loop_rate_limiters] [18:33:58] [warning] rate limiter is late by 4.6 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=27.985 ms, budget=20.000 ms, poll=0.078 ms, body_pose=6.054 ms, transform=9.304 ms, retarget=11.921 ms, skip_map=0.000 ms, min_height=0.066 ms, build_payload=0.034 ms, send=0.291 ms
[loop_rate_limiters] [18:33:58] [warning] rate limiter is late by 8.1 [ms] (rate_limiter.py:96)
[Warning] late frame breakdown: loop=23.606 ms, budget=20.000 ms, poll=0.096 ms, body_pose=6.182 ms, transform=6.311 ms, retarget=10.516 ms, skip_map=0.000 ms, min_height=0.057 ms, build_payload=0.020 ms, send=0.235 ms
[loop_rate_limiters] [18:33:59] [warning] rate limiter is late by 3.6 [ms] (rate_limiter.py:96)

##total_inference_cnt: 2100
	process_controllers: 0.018 ms
	get_low_state: 0.119 ms
	prepare_low_state: 0.148 ms
	prepare_obs: 0.466 ms
	policy: 0.882 ms
	rule_based_control_flow: 0.047 ms
total_inference_cnt: 2200
	process_controllers: 0.018 ms
	get_low_state: 0.145 ms
	prepare_low_state: 0.173 ms
	prepare_obs: 0.470 ms
	policy: 0.974 ms
	rule_based_control_flow: 0.062 ms
total_inference_cnt: 2300
	process_controllers: 0.019 ms
	get_low_state: 0.131 ms
	prepare_low_state: 0.163 ms
	prepare_obs: 0.529 ms
	policy: 1.109 ms
	rule_based_control_flow: 0.072 ms
total_inference_cnt: 2400
	process_controllers: 0.018 ms
	get_low_state: 0.122 ms
	prepare_low_state: 0.150 ms
	prepare_obs: 0.477 ms
	policy: 1.051 ms
	rule_based_control_flow: 0.064 ms
total_inference_cnt: 2500
	process_controllers: 0.017 ms
	get_low_state: 0.115 ms
	prepare_low_state: 0.142 ms
	prepare_obs: 0.444 ms
	policy: 1.041 ms
	rule_based_control_flow: 0.085 ms
total_inference_cnt: 2600
	process_controllers: 0.017 ms
	get_low_state: 0.116 ms
	prepare_low_state: 0.142 ms
	prepare_obs: 0.459 ms
	policy: 1.042 ms
	rule_based_control_flow: 0.056 ms
total_inference_cnt: 2700
	process_controllers: 0.019 ms
	get_low_state: 0.122 ms
	prepare_low_state: 0.152 ms
	prepare_obs: 0.474 ms
	policy: 1.093 ms
	rule_based_control_flow: 0.069 ms
total_inference_cnt: 2800
	process_controllers: 0.017 ms
	get_low_state: 0.111 ms
	prepare_low_state: 0.138 ms
	prepare_obs: 0.470 ms
	policy: 1.093 ms
	rule_based_control_flow: 0.062 ms



  total_inference_cnt: 700
	process_controllers: 0.017 ms
	get_low_state: 0.081 ms
	prepare_low_state: 0.106 ms
	prepare_obs: 0.371 ms
	policy: 2.112 ms
	rule_based_control_flow: 1.322 ms
total_inference_cnt: 800
	process_controllers: 0.019 ms
	get_low_state: 0.092 ms
	prepare_low_state: 0.119 ms
	prepare_obs: 0.424 ms
	policy: 2.175 ms
	rule_based_control_flow: 0.836 ms
total_inference_cnt: 900
	process_controllers: 0.016 ms
	get_low_state: 0.099 ms
	prepare_low_state: 0.123 ms
	prepare_obs: 0.398 ms
	policy: 2.195 ms
	rule_based_control_flow: 0.905 ms
total_inference_cnt: 1000
	process_controllers: 0.016 ms
	get_low_state: 0.087 ms
	prepare_low_state: 0.110 ms
	prepare_obs: 0.388 ms
	policy: 2.615 ms
	rule_based_control_flow: 0.687 ms


