# Security Policy

AUTO_ROVER phase 1 uses Ubuntu 20.04 and ROS 1 Noetic, both outside standard
upstream support. Deployments must pin the toolchain, base image, packages, and
upstream revisions. Before any networked Noetic deployment, complete a threat
assessment and contain or isolate the deployment appropriately.

Do not commit secrets, vehicle credentials, private keys, tokens, or production
connection details. Treat VCU protocol access and vehicle-control credentials as
sensitive operational data.

Report exploitable security vulnerabilities through [GitHub private vulnerability
reporting](https://github.com/niuma-phd/AUTO_ROVER/security/advisories/new).
Report non-sensitive software safety defects through a normal GitHub issue. This
policy covers software safety and security defects, but it does not make software
a replacement for the vehicle's independent hardware emergency-stop chain.
