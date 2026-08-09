# Repository Layout

The tree below is the approved target layout from the
[architecture baseline](../superpowers/specs/2026-08-10-auto-rover-architecture-design.md).
It is a create-on-demand map, not an instruction to create every path now. A
path is added only when it contains a reviewed deliverable needed by the active
delivery stage. Empty ROS packages, speculative adapters, and placeholder files
are prohibited.

```text
AUTO_ROVER/
├── README.md
├── ARCHITECTURE.md
├── ROADMAP.md
├── AGENTS.md
├── CONTRIBUTING.md
├── LICENSE
├── docs/
│   ├── architecture/
│   ├── interfaces/
│   ├── modules/
│   ├── vehicles/
│   ├── applications/
│   ├── adr/
│   └── testing/
├── src/
│   ├── core/auto_rover_core/
│   ├── interfaces/
│   │   ├── auto_rover_interfaces/
│   │   └── auto_rover_ros1_conversions/
│   ├── perception/auto_rover_localization/
│   ├── planning/auto_rover_planning/
│   ├── control/auto_rover_control/
│   ├── vehicle/
│   │   ├── auto_rover_vehicle/
│   │   └── adapters/auto_rover_vcu_<protocol>/
│   ├── safety/auto_rover_safety/
│   └── apps/
│       ├── known_map_navigation/auto_rover_known_map_bringup/
│       ├── autonomous_exploration/
│       └── field_coverage/
├── tools/
│   ├── route_tools/
│   ├── bag_tools/
│   └── visualization/
├── dependencies/
│   └── known_map.repos
├── tests/
│   ├── integration/
│   ├── replay/
│   ├── fixtures/
│   └── scenarios/
├── deploy/
│   ├── scripts/
│   ├── docker/
│   └── system/
└── .github/
    ├── workflows/
    ├── ISSUE_TEMPLATE/
    └── PULL_REQUEST_TEMPLATE.md
```

## Create-on-demand rules

- Only phase-1 packages are created initially, and only when their implementation
  work begins. Documentation directories likewise require maintained content.
- `LICENSE` is added only after the public-source license decision gate closes.
- The VCU adapter path is created only after the first protocol and feedback
  documentation are audited; `<protocol>` is not guessed.
- Autonomous exploration and field coverage remain future application
  compositions. Their directories are created only when their separately
  approved work begins.
- Tools, dependency manifests, tests, fixtures, deployment assets, and GitHub
  metadata are added when a concrete reviewed artifact requires their path.
- ROS 2 packages are not part of the phase-1 tree. Native ROS 2 work begins only
  after phase-1 validation and a supported target is selected.

## Dependency direction

```text
auto_rover_core -> algorithm core libraries
auto_rover_core + auto_rover_interfaces -> ROS 1 conversions and wrappers
algorithm cores + ROS 1 wrappers -> application bringup and integration
```

Lower-level packages never depend on an application. Planning never depends on
a localization implementation, control never depends on a VCU protocol package,
and a VCU adapter contains no trajectory-tracking algorithm.
