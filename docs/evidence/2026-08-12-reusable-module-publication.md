# Reusable module publication evidence

- Evidence date: 2026-08-12 (Asia/Shanghai)
- GitHub API observation time: `2026-08-11T17:45:48Z`
- Canonical repository: [`niuma-phd/AUTO_ROVER`](https://github.com/niuma-phd/AUTO_ROVER)
- Reviewed canonical source commit:
  [`9e45299872be3e6e1f68107bf35713d87c87196c`](https://github.com/niuma-phd/AUTO_ROVER/commit/9e45299872be3e6e1f68107bf35713d87c87196c)
- Governing publication decision:
  [ADR 0005](../adr/0005-apache-2.0-license-and-reusable-module-publication.md)
- Scope: post-push evidence for the six repositories whose final `main` HEAD
  has a successful GitHub Actions run
- Result: PASS for the six publications recorded below; this is not a vehicle
  release, package release, or physical-readiness result

All values below were resolved after the push. The local reviewed repository
HEAD, the GitHub `refs/heads/main` commit, and the Action `head_sha` matched for
each accepted publication. Repository metadata came from the GitHub REST API;
the commit, tree, and complete listing were independently resolved locally with
`git rev-parse HEAD`, `git rev-parse 'HEAD^{tree}'`, and
`git ls-tree -r --full-tree HEAD`.

## Publication summary

| Repository | Source-filter tip | Final `main` commit | Final tree | GitHub `pushed_at` | Accepted Action |
|---|---|---|---|---|---|
| [`auto-rover-core`](https://github.com/niuma-phd/auto-rover-core) | `31b87f6de49ee496fff78ee0a07834f4c79af894` | [`95764114943d82de2e75d9eec9d125b7c0f92800`](https://github.com/niuma-phd/auto-rover-core/commit/95764114943d82de2e75d9eec9d125b7c0f92800) | `1b6257ff563875c911d0b01a17c296c2c30fdb59` | `2026-08-11T16:58:52Z` | [run 31515226786](https://github.com/niuma-phd/auto-rover-core/actions/runs/31515226786), success |
| [`auto-rover-interfaces`](https://github.com/niuma-phd/auto-rover-interfaces) | `02f197e7941b3d482d52a11acc3e4a696740ab72` | [`511eafe4981109efdd73858d26726ebe182f3bdc`](https://github.com/niuma-phd/auto-rover-interfaces/commit/511eafe4981109efdd73858d26726ebe182f3bdc) | `7c77e950c2c2b9c15d9abcaa29fcbe12e9e3b0b5` | `2026-08-11T17:06:26Z` | [run 31515876070](https://github.com/niuma-phd/auto-rover-interfaces/actions/runs/31515876070), success |
| [`auto-rover-ros1-conversions`](https://github.com/niuma-phd/auto-rover-ros1-conversions) | `6075459009dc7b17caecaadd506f233ca1a9399c` | [`22a88a0857029d9f13842d611f5689d11a7c9905`](https://github.com/niuma-phd/auto-rover-ros1-conversions/commit/22a88a0857029d9f13842d611f5689d11a7c9905) | `82992f5f0a8f44793a892ad24ad32e29c1afd71d` | `2026-08-11T17:21:14Z` | [run 31517120210](https://github.com/niuma-phd/auto-rover-ros1-conversions/actions/runs/31517120210), success |
| [`auto-rover-safety`](https://github.com/niuma-phd/auto-rover-safety) | `c316ac5a8d1cab4de43f327466e924d162525b71` | [`6845b5bc39fb49117b2c026b250178ee662d48a7`](https://github.com/niuma-phd/auto-rover-safety/commit/6845b5bc39fb49117b2c026b250178ee662d48a7) | `4e895a186b79f5f28eeecb760325033d3b82a2a8` | `2026-08-11T17:11:00Z` | [run 31516264358](https://github.com/niuma-phd/auto-rover-safety/actions/runs/31516264358), success |
| [`auto-rover-control`](https://github.com/niuma-phd/auto-rover-control) | `9aad6d41a94c673e3e70b643042810f68947b8e4` | [`2a572d271e96dbdb578f8cbaaa2df4b26184cd97`](https://github.com/niuma-phd/auto-rover-control/commit/2a572d271e96dbdb578f8cbaaa2df4b26184cd97) | `350019802298357f48928c0f11ab128ac8302cef` | `2026-08-11T17:40:47Z` | [run 31518799131](https://github.com/niuma-phd/auto-rover-control/actions/runs/31518799131), success |
| [`auto-rover-vehicle`](https://github.com/niuma-phd/auto-rover-vehicle) | `a64f3b3c2bbc330f885c1a719a8aec0d92230334` | [`1ed23647e8bef4f32a1cdff70ac55d88dcc7fc21`](https://github.com/niuma-phd/auto-rover-vehicle/commit/1ed23647e8bef4f32a1cdff70ac55d88dcc7fc21) | `cf271bf5156af0d1724a52130da694d9da92cf84` | `2026-08-11T17:38:17Z` | [run 31518583722](https://github.com/niuma-phd/auto-rover-vehicle/actions/runs/31518583722), success |

The source-filter tip is the last commit produced by the reviewed allowlist
extraction before the repository-specific bootstrap files and any CI-only
bootstrap corrections were added. It is not the published `main` HEAD.

Earlier failed Actions remain visible in GitHub history. They were feedback
from the initial CI-bootstrap audit and exposed bootstrap-script or pinned
container/package alignment defects. Those defects were corrected in the final
commits above. Failed runs are deliberately not cited, counted, or treated as
acceptance evidence; acceptance is bound only to the successful run whose
`head_sha` is the final commit recorded in the table.

## Common repository and dependency facts

The GitHub API returned the following for every accepted repository:

- `visibility=public`, `default_branch=main`, and `license.spdx_id=Apache-2.0`;
- zero tags and zero releases;
- a root `LICENSE` with Git blob `261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64`
  and SHA-256
  `c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4`;
- exactly one exported package manifest, declaring `Apache-2.0`; and
- an active `Block incubation tags` ruleset targeting every tag ref, with one
  `creation` rule and no bypass actors.

The six package-specific lock files share this exact Ubuntu 20.04 amd64 / ROS
1 Noetic baseline:

```text
CI image: ros:noetic-ros-core-focal@sha256:4c1435fd85be3edde3820f0d132ab30a6b030209dce4fa3ac428a2aa5a763caf
ca-certificates=20240203~20.04.1
cmake=3.16.3-1ubuntu1.20.04.1
cpp-9=9.4.0-1ubuntu1~20.04.2
g++=4:9.3.0-1ubuntu2
g++-9=9.4.0-1ubuntu1~20.04.2
gcc-9=9.4.0-1ubuntu1~20.04.2
git=1:2.25.1-1ubuntu3.14
make=4.2.1-1.2
python3=3.8.2-0ubuntu2
python3.8=3.8.10-0ubuntu1~20.04.18
python3.8-minimal=3.8.10-0ubuntu1~20.04.18
ros-noetic-catkin=0.8.12-1focal.20250426.001935
```

Repository-specific dependency pins are recorded below. Every internal
AUTO_ROVER dependency is a full commit, never a floating branch.

| Repository | Additional external pins | Internal repository pins |
|---|---|---|
| `auto-rover-core` | None | None |
| `auto-rover-interfaces` | `ros-noetic-geometry-msgs=1.13.2-1focal.20250426.011953`; `ros-noetic-message-generation=0.4.1-1focal.20250426.010337`; `ros-noetic-message-runtime=0.4.13-1focal.20250426.011132`; `ros-noetic-rosmsg=1.17.4-1focal.20250519.234838`; `ros-noetic-std-msgs=0.5.14-1focal.20250426.011621` | None |
| `auto-rover-ros1-conversions` | `libgtest-dev=1.10.0-2`; `python3-vcstool=0.3.0-1`; `ros-noetic-geometry-msgs=1.13.2-1focal.20250426.011953`; `ros-noetic-message-generation=0.4.1-1focal.20250426.010337`; `ros-noetic-message-runtime=0.4.13-1focal.20250426.011132`; `ros-noetic-roscpp=1.17.4-1focal.20250519.225343`; `ros-noetic-std-msgs=0.5.14-1focal.20250426.011621` | core `95764114943d82de2e75d9eec9d125b7c0f92800`; interfaces `511eafe4981109efdd73858d26726ebe182f3bdc` |
| `auto-rover-safety` | `python3-vcstool=0.3.0-1` | core `95764114943d82de2e75d9eec9d125b7c0f92800` |
| `auto-rover-control` | `libgtest-dev=1.10.0-2`; `python3-vcstool=0.3.0-1`; `ros-noetic-geometry-msgs=1.13.2-1focal.20250426.011953`; `ros-noetic-message-generation=0.4.1-1focal.20250426.010337`; `ros-noetic-message-runtime=0.4.13-1focal.20250426.011132`; `ros-noetic-roscpp=1.17.4-1focal.20250519.225343`; `ros-noetic-std-msgs=0.5.14-1focal.20250426.011621` | core `95764114943d82de2e75d9eec9d125b7c0f92800`; interfaces `511eafe4981109efdd73858d26726ebe182f3bdc`; ROS 1 conversions `22a88a0857029d9f13842d611f5689d11a7c9905` |
| `auto-rover-vehicle` | `libgtest-dev=1.10.0-2`; `python3-vcstool=0.3.0-1`; `ros-noetic-geometry-msgs=1.13.2-1focal.20250426.011953`; `ros-noetic-message-generation=0.4.1-1focal.20250426.010337`; `ros-noetic-message-runtime=0.4.13-1focal.20250426.011132`; `ros-noetic-roscpp=1.17.4-1focal.20250519.225343`; `ros-noetic-std-msgs=0.5.14-1focal.20250426.011621`; `ros-noetic-std-srvs=1.11.4-1focal.20250426.011617` | core `95764114943d82de2e75d9eec9d125b7c0f92800`; interfaces `511eafe4981109efdd73858d26726ebe182f3bdc`; ROS 1 conversions `22a88a0857029d9f13842d611f5689d11a7c9905`; safety `6845b5bc39fb49117b2c026b250178ee662d48a7` |

## `auto-rover-core`

- Public URL: <https://github.com/niuma-phd/auto-rover-core>
- Canonical source commit: `9e45299872be3e6e1f68107bf35713d87c87196c`
- Filtered source tip: `31b87f6de49ee496fff78ee0a07834f4c79af894`
- Final commit: `95764114943d82de2e75d9eec9d125b7c0f92800`
- Final tree: `1b6257ff563875c911d0b01a17c296c2c30fdb59`
- GitHub `pushed_at`: `2026-08-11T16:58:52Z`
- Accepted Action: workflow `noetic-focal-amd64`, run
  [`31515226786`](https://github.com/niuma-phd/auto-rover-core/actions/runs/31515226786),
  `completed/success`, `head_sha=95764114943d82de2e75d9eec9d125b7c0f92800`
- Ruleset: ID `20706008`, `Block incubation tags`, `target=tag`,
  `enforcement=active`, include `~ALL`, exclude none, rule `creation`,
  `bypass_actors=[]`
- Repository state: public; default branch `main`; Apache-2.0; 0 tags; 0
  releases
- Internal dependencies: none

Complete `git ls-tree -r --full-tree HEAD`:

```text
100644 blob 9158cc09b8aed389341bc7b96b3f9c50283361c2	.github/workflows/noetic.yml
100644 blob 261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64	LICENSE
100644 blob b46bbcdec8b26464ecf312d299851be648933c8b	README.md
100644 blob 4ccbe8d46c36268559f72c8688d885c979766587	dependencies/noetic-focal-amd64.lock
100644 blob ef5bd812ad5baf20892ecdca44912849f6b88495	docs/contracts.md
100644 blob 61e08b1f78510ba913b0bf0970edddd4fc72fd24	docs/source-provenance.md
100644 blob e6c5fd554155e4210c88775a7bf404925138be2f	src/core/auto_rover_core/CMakeLists.txt
100644 blob 42113fba42b973743efc9e89b894796262d45a3e	src/core/auto_rover_core/include/auto_rover_core/geometry.hpp
100644 blob 7ac041a4952d2f0b0dd35e8fb33f4a658f36b140	src/core/auto_rover_core/include/auto_rover_core/types.hpp
100644 blob 2010cd3843102a3636c7e957cd41a38b4bd604f2	src/core/auto_rover_core/include/auto_rover_core/validation.hpp
100644 blob e763525b91fdc19cfbb2fb5af2cb18d1a624eeb7	src/core/auto_rover_core/package.xml
100644 blob 78809596b40c95408d9f5f573f61185013ee2733	src/core/auto_rover_core/src/geometry.cpp
100644 blob 0d1d1dbba3eadca6de32801ec0dd3f8824b9795f	src/core/auto_rover_core/src/validation.cpp
100644 blob 145b24c075158305048a9c26127411744caa3f81	tests/core/test_core.cpp
```

## `auto-rover-interfaces`

- Public URL: <https://github.com/niuma-phd/auto-rover-interfaces>
- Canonical source commit: `9e45299872be3e6e1f68107bf35713d87c87196c`
- Filtered source tip: `02f197e7941b3d482d52a11acc3e4a696740ab72`
- Final commit: `511eafe4981109efdd73858d26726ebe182f3bdc`
- Final tree: `7c77e950c2c2b9c15d9abcaa29fcbe12e9e3b0b5`
- GitHub `pushed_at`: `2026-08-11T17:06:26Z`
- Accepted Action: workflow `noetic-focal-amd64`, run
  [`31515876070`](https://github.com/niuma-phd/auto-rover-interfaces/actions/runs/31515876070),
  `completed/success`, `head_sha=511eafe4981109efdd73858d26726ebe182f3bdc`
- Ruleset: ID `20707446`, `Block incubation tags`, `target=tag`,
  `enforcement=active`, include `~ALL`, exclude none, rule `creation`,
  `bypass_actors=[]`
- Repository state: public; default branch `main`; Apache-2.0; 0 tags; 0
  releases
- Internal dependencies: none

Complete `git ls-tree -r --full-tree HEAD`:

```text
100644 blob 9f32d991d7ab195cdf8b7b16c460c8d614d1af68	.github/workflows/noetic.yml
100644 blob 261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64	LICENSE
100644 blob 835e0361570d22d843300ee8e4f2db0c87932e47	README.md
100644 blob cb763e42121ebb22e9596914eb623a88f3c4502a	dependencies/noetic-focal-amd64.lock
100644 blob 9e3e110aba2bc52d58e6d6124bd0b77fd3bba092	docs/contracts.md
100644 blob b06cebb0bd00f6a5a349342f1244996ed3c35925	docs/source-provenance.md
100644 blob 512c16f15e86f82e616837604b087563a8d8fb83	src/interfaces/auto_rover_interfaces/CMakeLists.txt
100644 blob c5dafbdbf25c2438737960e78b144bbbb44de721	src/interfaces/auto_rover_interfaces/msg/ChassisState.msg
100644 blob 16d833eaeed012393c957f4f22fe6184356a0163	src/interfaces/auto_rover_interfaces/msg/EgoState.msg
100644 blob 7284e480dacdf226fbb8a6c3e66a27459dd65435	src/interfaces/auto_rover_interfaces/msg/EmergencyStop.msg
100644 blob 1fcf714ec72a3b84004e84f15f30f822f530cb25	src/interfaces/auto_rover_interfaces/msg/MotionReference.msg
100644 blob f8388cb541420d12c9359fa679e073e41ba7bf66	src/interfaces/auto_rover_interfaces/msg/RoutePlan.msg
100644 blob a5a524101568e62c2ac3f5d9401f21f35915c1f5	src/interfaces/auto_rover_interfaces/msg/RouteWaypoint.msg
100644 blob a22c8d2e66e070ed9a8fd0642ba54f0a4873c37a	src/interfaces/auto_rover_interfaces/msg/SafetyState.msg
100644 blob d73ce444b973cdac0c4da20df2af0a03bcdb6efe	src/interfaces/auto_rover_interfaces/msg/Trajectory.msg
100644 blob 849b22f26c0ee88660c72e8c885d9f6eed25671f	src/interfaces/auto_rover_interfaces/msg/TrajectoryPoint.msg
100644 blob 132d78d3f51c56325ebe8b139476b12b60148a97	src/interfaces/auto_rover_interfaces/package.xml
100644 blob d951c2e1888e9853c52fdd488ebdcb63cfceb507	src/interfaces/auto_rover_interfaces/srv/ArmVehicle.srv
100644 blob e6cbcb7bab0683ae71f68cafb08a4aaeb0533b87	src/interfaces/auto_rover_interfaces/srv/AssertEmergencyStop.srv
100644 blob ad75e5a20760a1e3eec340de685a6715dc59d9a2	src/interfaces/auto_rover_interfaces/srv/ReloadRoute.srv
100644 blob 52d39cccd7b71459ebe80a640441b44d79b78547	src/interfaces/auto_rover_interfaces/srv/ResetEmergencyStop.srv
100644 blob 1e311bf83963618d23a69afb28c344dd5723720f	tests/interfaces/test_ros_contract_files.py
```

## `auto-rover-ros1-conversions`

- Public URL: <https://github.com/niuma-phd/auto-rover-ros1-conversions>
- Canonical source commit: `9e45299872be3e6e1f68107bf35713d87c87196c`
- Filtered source tip: `6075459009dc7b17caecaadd506f233ca1a9399c`
- Final commit: `22a88a0857029d9f13842d611f5689d11a7c9905`
- Final tree: `82992f5f0a8f44793a892ad24ad32e29c1afd71d`
- GitHub `pushed_at`: `2026-08-11T17:21:14Z`
- Accepted Action: workflow `noetic-focal-amd64`, run
  [`31517120210`](https://github.com/niuma-phd/auto-rover-ros1-conversions/actions/runs/31517120210),
  `completed/success`, `head_sha=22a88a0857029d9f13842d611f5689d11a7c9905`
- Ruleset: ID `20708072`, `Block incubation tags`, `target=tag`,
  `enforcement=active`, include `~ALL`, exclude none, rule `creation`,
  `bypass_actors=[]`
- Repository state: public; default branch `main`; Apache-2.0; 0 tags; 0
  releases
- Internal dependencies: core
  `95764114943d82de2e75d9eec9d125b7c0f92800`; interfaces
  `511eafe4981109efdd73858d26726ebe182f3bdc`

Complete `git ls-tree -r --full-tree HEAD`:

```text
100644 blob 7b085e45e9e8db281331493d2b5c47477a113b8e	.github/workflows/noetic.yml
100644 blob 261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64	LICENSE
100644 blob de5fda96559be980a7d9c4a351cfe2dd30f10302	README.md
100644 blob 1a9a320ab399ac9daf414c354a44c73ae76c2842	auto-rover.repos
100644 blob 7c0e09a6b6da096bed5ee585e1e486170e331f3f	dependencies/noetic-focal-amd64.lock
100644 blob 0f7d7900157e73302527d5f600bb09917f1321d3	docs/contracts.md
100644 blob d6b84b35805540b0d5330432c09ccfe055b8eebc	docs/source-provenance.md
100644 blob b6ebe15d5587db155fe1e602d2fddf2b991454d1	src/interfaces/auto_rover_ros1_conversions/CMakeLists.txt
100644 blob 8cfd4e90e7c673ff1559cbc92125adfe07cb56a6	src/interfaces/auto_rover_ros1_conversions/include/auto_rover_ros1_conversions/conversions.hpp
100644 blob f13e2f700e65c7bb12b598a6927a85478b04ad3f	src/interfaces/auto_rover_ros1_conversions/package.xml
100644 blob d23c584bb6cde1edcef2aa70df60ad78cd30718f	src/interfaces/auto_rover_ros1_conversions/src/conversions.cpp
100644 blob ce0e219d965b3349023fb8b65bb5ff7daf99d768	tests/interfaces/test_ros1_conversions.cpp
```

## `auto-rover-safety`

- Public URL: <https://github.com/niuma-phd/auto-rover-safety>
- Canonical source commit: `9e45299872be3e6e1f68107bf35713d87c87196c`
- Filtered source tip: `c316ac5a8d1cab4de43f327466e924d162525b71`
- Final commit: `6845b5bc39fb49117b2c026b250178ee662d48a7`
- Final tree: `4e895a186b79f5f28eeecb760325033d3b82a2a8`
- GitHub `pushed_at`: `2026-08-11T17:11:00Z`
- Accepted Action: workflow `noetic-focal-amd64`, run
  [`31516264358`](https://github.com/niuma-phd/auto-rover-safety/actions/runs/31516264358),
  `completed/success`, `head_sha=6845b5bc39fb49117b2c026b250178ee662d48a7`
- Ruleset: ID `20707755`, `Block incubation tags`, `target=tag`,
  `enforcement=active`, include `~ALL`, exclude none, rule `creation`,
  `bypass_actors=[]`
- Repository state: public; default branch `main`; Apache-2.0; 0 tags; 0
  releases
- Internal dependency: core
  `95764114943d82de2e75d9eec9d125b7c0f92800`

Complete `git ls-tree -r --full-tree HEAD`:

```text
100644 blob 804a95735cbab31b01c5f70347e340bd1fda39e5	.github/workflows/noetic.yml
100644 blob 261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64	LICENSE
100644 blob b5c46e44dded959d6c0d6ddeb9b853a79fc5eeda	README.md
100644 blob 4309f96d72a7aad829f6c7b220a2a5c65fd2fa61	auto-rover.repos
100644 blob 8e525f32aba0200372092ceaa3be9ac7ddcceb1c	dependencies/noetic-focal-amd64.lock
100644 blob d6e36a8cf3971f628c0b0490a84bb126d92b2e69	docs/contracts.md
100644 blob b2ba2b12b3ab2c4c2d55f645422e6a2a23acd1ed	docs/source-provenance.md
100644 blob d6bffd71cd6148cca1608fe20f6adada3a7f0dd8	src/safety/auto_rover_safety/CMakeLists.txt
100644 blob b29dd14b4d6d1ec39724402ae45a5a66c0677625	src/safety/auto_rover_safety/include/auto_rover_safety/command_guard.hpp
100644 blob 3c409e4f6cfe3df60632f69fcd242b15c66e662d	src/safety/auto_rover_safety/include/auto_rover_safety/safety_supervisor.hpp
100644 blob 2866726384ec569833cfe009787c736570c40e0e	src/safety/auto_rover_safety/package.xml
100644 blob f294a3fcc6ef5b179afa6b4e038a4dc501cb7eaa	src/safety/auto_rover_safety/src/command_guard.cpp
100644 blob a92cfb3e09e01bb4b91a56e919f31bac1b7ac01f	src/safety/auto_rover_safety/src/safety_supervisor.cpp
100644 blob 0bd61dc334685f8f8f52423008ea536884346c51	tests/safety/test_safety.cpp
```

## `auto-rover-control`

- Public URL: <https://github.com/niuma-phd/auto-rover-control>
- Canonical source commit: `9e45299872be3e6e1f68107bf35713d87c87196c`
- Filtered source tip: `9aad6d41a94c673e3e70b643042810f68947b8e4`
- Final commit: `2a572d271e96dbdb578f8cbaaa2df4b26184cd97`
- Final tree: `350019802298357f48928c0f11ab128ac8302cef`
- GitHub `pushed_at`: `2026-08-11T17:40:47Z`
- Accepted Action: workflow `noetic-focal-amd64`, run
  [`31518799131`](https://github.com/niuma-phd/auto-rover-control/actions/runs/31518799131),
  `completed/success`, `head_sha=2a572d271e96dbdb578f8cbaaa2df4b26184cd97`
- Ruleset: ID `20708641`, `Block incubation tags`, `target=tag`,
  `enforcement=active`, include `~ALL`, exclude none, rule `creation`,
  `bypass_actors=[]`
- Repository state: public; default branch `main`; Apache-2.0; 0 tags; 0
  releases
- Internal dependencies: core
  `95764114943d82de2e75d9eec9d125b7c0f92800`; interfaces
  `511eafe4981109efdd73858d26726ebe182f3bdc`; ROS 1 conversions
  `22a88a0857029d9f13842d611f5689d11a7c9905`

Complete `git ls-tree -r --full-tree HEAD`:

```text
100644 blob 40f4f51ebbdbdf943c68349f3cb8c040445ef94d	.github/workflows/noetic.yml
100644 blob 261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64	LICENSE
100644 blob 1ecdfc43b5c9d9ccd7c4d0e52ac1b9c479a80fbe	README.md
100644 blob 1c6865e58afd47b9c534be0ecace496daba3cda3	auto-rover.repos
100644 blob 7c0e09a6b6da096bed5ee585e1e486170e331f3f	dependencies/noetic-focal-amd64.lock
100644 blob c5a39c8ea8e62668d03071996994007e96e14790	docs/contracts.md
100644 blob afdf1eb6628363a5d2ea4a6f011d4dfba52e347e	docs/source-provenance.md
100644 blob f4422b21e09fd9315f637da70a1d928bc2e5775c	src/control/auto_rover_control/CMakeLists.txt
100644 blob 2e9dfc5c223e7b6437e0a07ecd5bfbe7f2806225	src/control/auto_rover_control/include/auto_rover_control/pure_pursuit.hpp
100644 blob 1a3f4f8306efdfbf59ef42965b3100cc39722652	src/control/auto_rover_control/package.xml
100644 blob 449dfd97745157c7f56ab1ff98eb2307b4a8a74d	src/control/auto_rover_control/src/pure_pursuit.cpp
100644 blob 84cddec6f82c302e9cfd23b538826bceb3c82884	src/control/auto_rover_control/src/pure_pursuit_node.cpp
100644 blob 29450cd8fbb5aa42e580eaf0daf96d402403d1c0	tests/control/test_pure_pursuit.cpp
100644 blob 38d8affae4a07e25f7b194350238f5aae0967ca4	tests/ros_wrappers/test_control_wrapper_contract.py
```

## `auto-rover-vehicle`

- Public URL: <https://github.com/niuma-phd/auto-rover-vehicle>
- Canonical source commit: `9e45299872be3e6e1f68107bf35713d87c87196c`
- Filtered source tip: `a64f3b3c2bbc330f885c1a719a8aec0d92230334`
- Final commit: `1ed23647e8bef4f32a1cdff70ac55d88dcc7fc21`
- Final tree: `cf271bf5156af0d1724a52130da694d9da92cf84`
- GitHub `pushed_at`: `2026-08-11T17:38:17Z`
- Accepted Action: workflow `noetic-focal-amd64`, run
  [`31518583722`](https://github.com/niuma-phd/auto-rover-vehicle/actions/runs/31518583722),
  `completed/success`, `head_sha=1ed23647e8bef4f32a1cdff70ac55d88dcc7fc21`
- Ruleset: ID `20708518`, `Block incubation tags`, `target=tag`,
  `enforcement=active`, include `~ALL`, exclude none, rule `creation`,
  `bypass_actors=[]`
- Repository state: public; default branch `main`; Apache-2.0; 0 tags; 0
  releases
- Internal dependencies: core
  `95764114943d82de2e75d9eec9d125b7c0f92800`; interfaces
  `511eafe4981109efdd73858d26726ebe182f3bdc`; ROS 1 conversions
  `22a88a0857029d9f13842d611f5689d11a7c9905`; safety
  `6845b5bc39fb49117b2c026b250178ee662d48a7`

Complete `git ls-tree -r --full-tree HEAD`:

```text
100644 blob cc8b79e73f10687e9ba3e154fee32eaf91080430	.github/workflows/noetic.yml
100644 blob 261eeb9e9f8b2b4b0d119366dda99c6fd7d35c64	LICENSE
100644 blob 5a120957bae73bf1202e8fde61d24b0f8459fcf5	README.md
100644 blob 972ed931a71bd83e0f7378a293114913b2f36d8c	auto-rover.repos
100644 blob c573fc8e17aefb6546a3716acba919f58de0e431	dependencies/noetic-focal-amd64.lock
100644 blob aa4106b3c2b648f1c9cbe634174eaf00091be124	docs/contracts.md
100644 blob 11bf2a653aaf0eacaa0f034b030fbad926d31b5e	docs/source-provenance.md
100644 blob b9e578dd901c694fe46371ed618ffa57b76929ad	src/vehicle/auto_rover_vehicle/CMakeLists.txt
100644 blob e54498d067511c3fa6e7f7b69125b45d450431f4	src/vehicle/auto_rover_vehicle/include/auto_rover_vehicle/fake_vcu.hpp
100644 blob c7e7d42d21d723648fe93cf89b81c5e39d56bd5b	src/vehicle/auto_rover_vehicle/include/auto_rover_vehicle/vehicle_backend.hpp
100644 blob 7a1314ee39b67f557b68f5474823b78ee55434ff	src/vehicle/auto_rover_vehicle/include/auto_rover_vehicle/vehicle_execution_core.hpp
100644 blob 5910debb1926288199cb4802c1a3993d00424cc5	src/vehicle/auto_rover_vehicle/include/auto_rover_vehicle/vehicle_motion_manager.hpp
100644 blob 925a8d271e58b4e97fee85a8b9d9335bc9f9ae28	src/vehicle/auto_rover_vehicle/package.xml
100644 blob c9a9f13ed5663055ca4c878d032ef25ba696a22d	src/vehicle/auto_rover_vehicle/src/fake_vcu.cpp
100644 blob 2d654d453e9446a35370a2b7395ab37be9ad4ad1	src/vehicle/auto_rover_vehicle/src/vehicle_execution_core.cpp
100644 blob b3412d5da173a157ceeb8e99d4d63be1e36df3be	src/vehicle/auto_rover_vehicle/src/vehicle_execution_node.cpp
100644 blob e2e64930a267ff63b15ce080256b9bf6bf02c2da	src/vehicle/auto_rover_vehicle/src/vehicle_motion_manager.cpp
100644 blob a04880f1df61b5440070536c58b168f3cbff6595	tests/vehicle/test_vehicle.cpp
100644 blob 0a63def1569eb3c9f83c370e5620bd21442ec0dc	tests/vehicle_ros/test_vehicle_execution_core.cpp
100644 blob 53d053abf42cf52ba69b22d425fdb84eaba6c886	tests/vehicle_ros/test_vehicle_execution_node_contract.py
```

## Remaining publication slots

The approved manifest contains seven repositories. Six of seven now have the
accepted post-push evidence above. The seventh, planning, remains blocked on
its safety-contract decision.

### `auto-rover-planning` — Blocked

[ADR 0006](../adr/0006-bounded-known-map-planning-resources.md) is still
`Proposed`; its explicit repository-owner acceptance evidence is pending.
Planning publication remains blocked until that decision is accepted and the
export is regenerated and reviewed against the accepted canonical commit. This
record intentionally asserts no public URL or post-push fact for planning.

## Safety and authority boundary

Publishing reusable source does not change the physical gate. The compile-time
physical-actuation release gate remains closed, and no extracted repository is
an authority to open a serial command channel. Installed-firmware identity,
parser resynchronization after an unclean session, external power isolation,
device ownership, independent watchdog behavior, ground braking/hold, and the
later live FAST-LIVO2 acceptance remain separate, unclosed vehicle evidence.

AUTO_ROVER remains the canonical integration and vehicle-acceptance source.
These repositories are commit-pinned incubation source: they have no tags, no
releases, no independent release authority, and no vehicle-safety acceptance.
