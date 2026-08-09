import contextlib
import io
import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from tools.ci.validate_repository import Finding, main, validate_repository


REQUIRED_TEXT_FILES = (
    ".editorconfig",
    ".gitattributes",
    ".gitignore",
    "AGENTS.md",
    "ARCHITECTURE.md",
    "CONTRIBUTING.md",
    "README.md",
    "ROADMAP.md",
    "SECURITY.md",
    ".github/PULL_REQUEST_TEMPLATE.md",
    ".github/ISSUE_TEMPLATE/bug-report.yml",
    ".github/ISSUE_TEMPLATE/change-proposal.yml",
    ".github/ISSUE_TEMPLATE/config.yml",
    ".github/workflows/repository-validation.yml",
    "docs/README.md",
    "docs/adr/0000-template.md",
    "docs/adr/README.md",
    "docs/architecture/architecture-policy.json",
    "docs/architecture/repository-layout.md",
    "docs/governance/dependencies-and-licensing.md",
    "docs/interfaces/contracts.md",
    "docs/superpowers/specs/2026-08-10-auto-rover-architecture-design.md",
    "docs/testing/phase-1-acceptance.md",
    "docs/vehicles/vcu-adapter-readiness.md",
    "tests/repository/test_validate_repository.py",
    "tools/ci/validate_repository.py",
)


ARCHITECTURE_POLICY = {
    "schema_version": 1,
    "repository_strategy": "monorepo-first",
    "autonomy_layers": ["perception", "planning", "control"],
    "phase_1": {
        "runtime": {"os": "ubuntu-20.04", "middleware": "ros1-noetic"},
        "application": "known-map-navigation",
        "deployment_model": "single-purpose",
        "route_generation": "off-board",
        "onboard_planning": "route-plan-to-trajectory",
        "task_manager": False,
        "world_model": "reserved-boundary",
        "local_obstacle_avoidance": False,
    },
    "migration": {
        "ros2": "native-after-phase-1",
        "dual_runtime_phase_1": False,
    },
    "contracts": {
        "localization": "EgoState",
        "route": "RoutePlan",
        "trajectory": "Trajectory",
        "control_output": "MotionReference",
        "chassis_feedback": "ChassisState",
    },
    "vehicle_integration": {
        "universal_cmd_vel": False,
        "vcu_adapter_required": True,
        "chassis_feedback_required": True,
        "preserve_existing_vcu_inner_loop": True,
    },
}


ISSUE_FORM = """name: Example
description: Example form
title: \"[Example]: \"
labels:
  - example
body:
  - type: textarea
    id: description
    attributes:
      label: Description
    validations:
      required: true
"""


ISSUE_CONFIG = """blank_issues_enabled: false
contact_links:
  - name: Security
    url: https://example.com/security
    about: Report privately
"""


WORKFLOW = """name: Repository validation
on:
  pull_request:
  push:
    branches:
      - main
permissions:
  contents: read
concurrency:
  group: repository-validation-${{ github.ref }}
  cancel-in-progress: true
jobs:
  required:
    name: required
    runs-on: ubuntu-24.04
    timeout-minutes: 5
    steps:
      - name: Check out repository
        uses: actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1
      - name: Set up Python
        uses: actions/setup-python@5fda3b95a4ea91299a34e894583c3862153e4b97
        with:
          python-version: \"3.8.18\"
      - name: Run validator unit tests
        run: python -m unittest discover -s tests/repository -p \"test_*.py\" -v
      - name: Validate repository
        run: python tools/ci/validate_repository.py --root .
"""


def write_text(root, relative_path, content):
    path = root / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def make_valid_repository(root):
    for relative_path in REQUIRED_TEXT_FILES:
        write_text(root, relative_path, "# Repository fixture\n")
    write_text(
        root,
        "docs/architecture/architecture-policy.json",
        json.dumps(ARCHITECTURE_POLICY, indent=2) + "\n",
    )
    write_text(root, ".github/ISSUE_TEMPLATE/bug-report.yml", ISSUE_FORM)
    write_text(root, ".github/ISSUE_TEMPLATE/change-proposal.yml", ISSUE_FORM)
    write_text(root, ".github/ISSUE_TEMPLATE/config.yml", ISSUE_CONFIG)
    write_text(root, ".github/workflows/repository-validation.yml", WORKFLOW)


class RepositoryValidatorTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        make_valid_repository(self.root)

    def findings(self, code):
        return [item for item in validate_repository(self.root) if item.code == code]

    def test_valid_fixture_has_no_findings(self):
        self.assertEqual([], validate_repository(self.root))

    def test_missing_empty_and_wrong_case_required_files_are_reported(self):
        (self.root / "README.md").unlink()
        (self.root / "ARCHITECTURE.md").write_text("", encoding="utf-8")
        (self.root / "SECURITY.md").rename(self.root / "security.md")

        findings = validate_repository(self.root)

        self.assertTrue(any(item.code == "REQUIRED_MISSING" and item.path == "README.md" for item in findings))
        self.assertTrue(any(item.code == "REQUIRED_EMPTY" and item.path == "ARCHITECTURE.md" for item in findings))
        self.assertTrue(any(item.code == "REQUIRED_CASE" and item.path == "SECURITY.md" for item in findings))

    @unittest.skipIf(os.name == "nt", "case-colliding files cannot be created on Windows")
    def test_case_fold_path_collisions_are_reported(self):
        write_text(self.root, "readme.md", "# Colliding name\n")

        findings = [
            item for item in validate_repository(self.root) if item.code == "PATH_CASE_COLLISION"
        ]

        self.assertEqual(["README.md", "readme.md"], [item.path for item in findings])

    @unittest.skipIf(os.name == "nt", "symlink behavior is covered by Linux CI")
    def test_required_directory_symlink_is_rejected(self):
        mirror = self.root / "mirror-architecture"
        write_text(
            self.root,
            "mirror-architecture/architecture-policy.json",
            json.dumps(ARCHITECTURE_POLICY) + "\n",
        )
        shutil.rmtree(self.root / "docs/architecture")
        (self.root / "docs/architecture").symlink_to(mirror, target_is_directory=True)

        findings = validate_repository(self.root)

        self.assertTrue(
            any(
                item.path == "docs/architecture" and item.code == "PATH_SYMLINK"
                for item in findings
            )
        )

    @unittest.skipIf(os.name == "nt", "symlink behavior is covered by Linux CI")
    def test_escaping_required_file_symlink_is_rejected(self):
        outside_directory = tempfile.TemporaryDirectory()
        self.addCleanup(outside_directory.cleanup)
        outside = Path(outside_directory.name) / "README.md"
        outside.write_text("# Outside\n", encoding="utf-8")
        (self.root / "README.md").unlink()
        (self.root / "README.md").symlink_to(outside)

        findings = validate_repository(self.root)

        self.assertTrue(
            any(
                item.path == "README.md" and item.code == "PATH_SYMLINK_ESCAPE"
                for item in findings
            )
        )

    def test_non_utf8_first_party_text_is_reported(self):
        (self.root / "notes.md").write_bytes(b"\xff\xfe")

        findings = validate_repository(self.root)

        self.assertEqual(["TEXT_ENCODING"], [item.code for item in findings])

    def test_prohibited_markers_ignore_generated_and_plan_directories(self):
        marker = "TO" + "DO"
        write_text(self.root, "notes.md", marker + ": replace this\n")
        excluded = (
            ".git",
            ".catkin_tools",
            ".venv",
            "build",
            "devel",
            "env",
            "generated",
            "install",
            "log",
            "third_party",
            "venv",
            "docs/superpowers/plans",
        )
        for directory in excluded:
            write_text(self.root, directory + "/ignored.md", marker + ": ignored\n")

        findings = self.findings("PLACEHOLDER")

        self.assertEqual(1, len(findings))
        self.assertEqual("notes.md", findings[0].path)

    def test_nested_curated_directories_are_not_blanket_excluded(self):
        marker = "TO" + "DO"
        write_text(self.root, "docs/env/README.md", marker + ": checked\n")
        write_text(self.root, "src/build/notes.md", marker + ": checked\n")

        findings = self.findings("PLACEHOLDER")

        self.assertEqual(
            ["docs/env/README.md", "src/build/notes.md"],
            [item.path for item in findings],
        )

    def test_planned_ros_and_deployment_text_formats_are_scanned(self):
        marker = "TO" + "DO"
        paths = (
            "dependencies/example.repos",
            "src/robot.urdf",
            "src/robot.xacro",
            "config/view.rviz",
            "pyproject.toml",
            "deploy/Dockerfile",
        )
        for relative_path in paths:
            write_text(self.root, relative_path, marker + ": checked\n")

        findings = self.findings("PLACEHOLDER")

        self.assertEqual(sorted(paths), [item.path for item in findings])

    def test_valid_percent_encoded_relative_link_and_image_resolve(self):
        write_text(self.root, "docs/target file.md", "# Target\n")
        write_text(
            self.root,
            "docs/source.md",
            "[target](target%20file.md#section)\n![image](../image.png)\n",
        )
        (self.root / "image.png").write_bytes(b"image")

        self.assertEqual([], validate_repository(self.root))

    def test_missing_escaping_and_wrong_case_links_are_reported(self):
        write_text(self.root, "docs/existing.md", "# Existing\n")
        write_text(
            self.root,
            "docs/links.md",
            "[missing](missing.md)\n[escape](../../outside.md)\n[case](Existing.md)\n",
        )

        findings = validate_repository(self.root)
        codes = [item.code for item in findings if item.path == "docs/links.md"]

        self.assertEqual(["LINK_CASE", "LINK_ESCAPE", "LINK_MISSING"], sorted(codes))

    def test_windows_absolute_link_is_reported_as_repository_escape(self):
        write_text(self.root, "docs/windows-link.md", "[outside](C:/outside.md)\n")

        findings = validate_repository(self.root)

        self.assertTrue(
            any(
                item.path == "docs/windows-link.md" and item.code == "LINK_ESCAPE"
                for item in findings
            )
        )

    def test_reference_style_link_definition_target_is_validated(self):
        write_text(
            self.root,
            "docs/reference.md",
            "[missing][reference]\n\n[reference]: missing.md\n",
        )

        findings = validate_repository(self.root)

        self.assertTrue(
            any(
                item.path == "docs/reference.md" and item.code == "LINK_MISSING"
                for item in findings
            )
        )

    def test_github_footnote_definition_is_not_treated_as_a_link_target(self):
        write_text(
            self.root,
            "docs/footnote.md",
            "A statement with a note.[^1]\n\n[^1]: This is a note.\n",
        )

        self.assertEqual([], validate_repository(self.root))

    def test_balanced_escaped_angle_multiline_links_and_inline_code(self):
        write_text(self.root, "docs/target_(v1).md", "# Target\n")
        write_text(
            self.root,
            "docs/commonmark-links.md",
            "`[ignored](missing.md)`\n"
            "[balanced](target_(v1).md)\n"
            "[escaped](target_\\(v1\\).md)\n"
            "[angle](<target_(v1).md>)\n"
            "[multiline](\n  target_(v1).md\n)\n",
        )

        self.assertEqual([], validate_repository(self.root))

    def test_markdown_requires_a_real_label_and_validates_titles_and_spaced_destinations(self):
        write_text(self.root, "docs/target file.md", "# Target\n")
        write_text(
            self.root,
            "docs/link-boundaries.md",
            "prose](not-a-link.md)\n"
            "[missing](missing.md (title))\n"
            "[space](<target file.md>)\n",
        )

        findings = [
            item
            for item in validate_repository(self.root)
            if item.path == "docs/link-boundaries.md"
        ]

        self.assertEqual(["LINK_MISSING"], [item.code for item in findings])
        self.assertIn("missing.md", findings[0].message)

    def test_invalid_backtick_fence_info_does_not_hide_a_broken_link(self):
        write_text(
            self.root,
            "docs/invalid-fence.md",
            "```markdown`invalid\n[missing](missing.md)\n```\n",
        )

        codes = [
            item.code
            for item in validate_repository(self.root)
            if item.path == "docs/invalid-fence.md"
        ]

        self.assertIn("MARKDOWN_FENCE", codes)
        self.assertIn("LINK_MISSING", codes)

    def test_links_inside_fenced_code_are_ignored(self):
        write_text(
            self.root,
            "docs/example.md",
            "```markdown\n[not a link](missing.md)\n```\n",
        )

        self.assertEqual([], validate_repository(self.root))

    def test_paired_backtick_and_tilde_fences_are_valid(self):
        write_text(
            self.root,
            "docs/fences.md",
            "````text\n``` nested\n````\n~~~text\ncontent\n~~~\n",
        )

        self.assertEqual([], validate_repository(self.root))

    def test_unpaired_backtick_and_tilde_fences_are_reported(self):
        write_text(self.root, "docs/backtick.md", "```text\ncontent\n")
        write_text(self.root, "docs/tilde.md", "~~~text\ncontent\n")

        findings = self.findings("MARKDOWN_FENCE")

        self.assertEqual(["docs/backtick.md", "docs/tilde.md"], [item.path for item in findings])

    def test_malformed_json_is_reported(self):
        write_text(self.root, "docs/architecture/architecture-policy.json", "{broken\n")

        self.assertEqual(["JSON_INVALID"], [item.code for item in validate_repository(self.root)])

    def test_duplicate_json_keys_are_rejected(self):
        write_text(
            self.root,
            "docs/architecture/architecture-policy.json",
            '{"schema_version": 1, "schema_version": 1}\n',
        )

        self.assertEqual(["JSON_DUPLICATE_KEY"], [item.code for item in validate_repository(self.root)])

    def test_non_standard_json_constants_are_rejected(self):
        write_text(self.root, "docs/extra.json", '{"value": NaN}\n')

        findings = [
            item
            for item in validate_repository(self.root)
            if item.path == "docs/extra.json"
        ]

        self.assertEqual(["JSON_INVALID"], [item.code for item in findings])

    def test_changed_or_missing_required_policy_values_are_reported(self):
        policy = json.loads(json.dumps(ARCHITECTURE_POLICY))
        policy["phase_1"]["task_manager"] = True
        del policy["contracts"]["trajectory"]
        write_text(
            self.root,
            "docs/architecture/architecture-policy.json",
            json.dumps(policy) + "\n",
        )

        findings = self.findings("POLICY_VALUE")

        self.assertEqual(2, len(findings))
        self.assertTrue(any("phase_1.task_manager" in item.message for item in findings))
        self.assertTrue(any("contracts.trajectory" in item.message for item in findings))

    def test_policy_values_require_exact_json_types(self):
        policy = json.loads(json.dumps(ARCHITECTURE_POLICY))
        policy["schema_version"] = True
        policy["phase_1"]["task_manager"] = 0
        policy["vehicle_integration"]["vcu_adapter_required"] = 1
        write_text(
            self.root,
            "docs/architecture/architecture-policy.json",
            json.dumps(policy) + "\n",
        )

        findings = self.findings("POLICY_VALUE")

        self.assertEqual(3, len(findings))
        self.assertTrue(any("schema_version" in item.message for item in findings))
        self.assertTrue(any("phase_1.task_manager" in item.message for item in findings))
        self.assertTrue(
            any("vehicle_integration.vcu_adapter_required" in item.message for item in findings)
        )

    def test_issue_forms_require_top_level_keys_and_unique_body_ids(self):
        form = ISSUE_FORM.replace("description: Example form\n", "")
        form += "  - type: input\n    id: description\n"
        write_text(self.root, ".github/ISSUE_TEMPLATE/bug-report.yml", form)

        findings = validate_repository(self.root)
        codes = [item.code for item in findings]

        self.assertIn("ISSUE_FORM_KEY", codes)
        self.assertIn("ISSUE_FORM_DUPLICATE_ID", codes)

    def test_yaml_tabs_bad_indentation_and_duplicate_top_level_keys_are_reported(self):
        write_text(
            self.root,
            ".github/ISSUE_TEMPLATE/change-proposal.yml",
            ISSUE_FORM + "name: Duplicate\n   invalid: value\n\tbad: value\n",
        )

        codes = [item.code for item in validate_repository(self.root)]

        self.assertIn("YAML_DUPLICATE_KEY", codes)
        self.assertIn("YAML_INDENT", codes)
        self.assertIn("YAML_TAB", codes)

    def test_workflow_requires_top_level_keys_and_required_job_shape(self):
        workflow = WORKFLOW.replace("permissions:\n  contents: read\n", "")
        workflow = workflow.replace(
            "concurrency:\n  group: repository-validation-${{ github.ref }}\n  cancel-in-progress: true\n",
            "",
        )
        workflow = workflow.replace("    timeout-minutes: 5\n", "")
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        findings = validate_repository(self.root)
        codes = [item.code for item in findings]

        self.assertIn("WORKFLOW_KEY", codes)
        self.assertIn("WORKFLOW_JOB", codes)

    def test_workflow_requires_pinned_python_and_validation_commands(self):
        workflow = WORKFLOW.replace('          python-version: "3.8.18"\n', "")
        workflow = workflow.replace(
            "python tools/ci/validate_repository.py --root .",
            "python different_validator.py",
        )
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        messages = [
            item.message
            for item in validate_repository(self.root)
            if item.code == "WORKFLOW_JOB"
        ]

        self.assertTrue(any("Python 3.8.18" in message for message in messages))
        self.assertTrue(any("validate_repository.py" in message for message in messages))

    def test_workflow_requires_concurrency_cancellation_shape(self):
        concurrency = """concurrency:
  group: repository-validation-${{ github.ref }}
  cancel-in-progress: true
"""
        write_text(
            self.root,
            ".github/workflows/repository-validation.yml",
            WORKFLOW.replace(concurrency, ""),
        )

        codes = [item.code for item in validate_repository(self.root)]

        self.assertIn("WORKFLOW_KEY", codes)
        self.assertIn("WORKFLOW_CONCURRENCY", codes)

    def test_workflow_events_hidden_in_literal_block_do_not_count(self):
        event_block = """name: Repository validation
on:
  pull_request:
  push:
    branches:
      - main
"""
        hidden_manual_only = """name: |
  Repository validation
  pull_request:
  push:
    branches:
      - main
on:
  workflow_dispatch:
"""
        workflow = WORKFLOW.replace(event_block, hidden_manual_only)
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        self.assertIn(
            "WORKFLOW_EVENT",
            [item.code for item in validate_repository(self.root)],
        )

    def test_workflow_rejects_quoted_unsafe_keys(self):
        probes = (
            WORKFLOW.replace(
                "  pull_request:\n",
                '  pull_request:\n  "pull_request_target":\n',
            ),
            WORKFLOW.replace(
                "  required:\n",
                '  required:\n    "permissions": write-all\n',
            ),
        )
        expected_codes = ("WORKFLOW_UNSAFE_EVENT", "WORKFLOW_PERMISSIONS")
        for workflow, expected_code in zip(probes, expected_codes):
            with self.subTest(expected_code=expected_code):
                write_text(
                    self.root,
                    ".github/workflows/repository-validation.yml",
                    workflow,
                )
                self.assertIn(
                    expected_code,
                    [item.code for item in validate_repository(self.root)],
                )

    def test_workflow_rejects_duplicate_canonical_quoted_keys(self):
        workflow = WORKFLOW.replace(
            "  contents: read\n",
            '  contents: write\n  "contents": read\n',
        )
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        self.assertIn(
            "YAML_DUPLICATE_KEY",
            [item.code for item in validate_repository(self.root)],
        )

    def test_workflow_pull_request_trigger_must_be_unconditional(self):
        filters = (
            "    branches-ignore:\n      - main\n",
            "    types:\n      - closed\n",
            "    paths-ignore:\n      - '**'\n",
        )
        for trigger_filter in filters:
            with self.subTest(trigger_filter=trigger_filter):
                workflow = WORKFLOW.replace(
                    "  pull_request:\n",
                    "  pull_request:\n" + trigger_filter,
                )
                write_text(
                    self.root,
                    ".github/workflows/repository-validation.yml",
                    workflow,
                )
                self.assertIn(
                    "WORKFLOW_EVENT",
                    [item.code for item in validate_repository(self.root)],
                )

    def test_workflow_commands_hidden_after_echo_comment_do_not_count(self):
        workflow = WORKFLOW.replace(
            '        run: python -m unittest discover -s tests/repository -p "test_*.py" -v',
            '        run: echo bypass # python -m unittest discover -s tests/repository -p "test_*.py" -v',
        )
        workflow = workflow.replace(
            "        run: python tools/ci/validate_repository.py --root .",
            "        run: echo bypass # python tools/ci/validate_repository.py --root .",
        )
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        self.assertIn(
            "WORKFLOW_JOB",
            [item.code for item in validate_repository(self.root)],
        )

    def test_workflow_rejects_unpinned_extra_action(self):
        workflow = WORKFLOW + """      - name: Unpinned action
        uses: actions/github-script@main
"""
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        self.assertIn(
            "WORKFLOW_ACTION_PIN",
            [item.code for item in validate_repository(self.root)],
        )

    def test_workflow_rejects_unsafe_event_and_permissions(self):
        workflow = WORKFLOW.replace("  pull_request:\n", "  pull_request_target:\n")
        workflow = workflow.replace("  contents: read\n", "  contents: write\n")
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        findings = validate_repository(self.root)
        codes = [item.code for item in findings]

        self.assertIn("WORKFLOW_UNSAFE_EVENT", codes)
        self.assertIn("WORKFLOW_PERMISSIONS", codes)

    def test_workflow_rejects_write_all_permissions(self):
        workflow = WORKFLOW.replace("permissions:\n  contents: read\n", "permissions: write-all\n")
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        self.assertIn("WORKFLOW_PERMISSIONS", [item.code for item in validate_repository(self.root)])

    def test_workflow_rejects_job_level_write_all_with_comment(self):
        workflow = WORKFLOW.replace(
            "  required:\n",
            "  required:\n    permissions: write-all # unsafe override\n",
        )
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        self.assertIn(
            "WORKFLOW_PERMISSIONS",
            [item.code for item in validate_repository(self.root)],
        )

    def test_workflow_rejects_job_level_write_scope(self):
        workflow = WORKFLOW.replace(
            "  required:\n",
            "  required:\n    permissions:\n      issues: write\n",
        )
        write_text(self.root, ".github/workflows/repository-validation.yml", workflow)

        self.assertIn(
            "WORKFLOW_PERMISSIONS",
            [item.code for item in validate_repository(self.root)],
        )

    def test_findings_have_deterministic_dataclass_order(self):
        marker = "FIX" + "ME"
        write_text(self.root, "zeta.md", marker + "\n")
        write_text(self.root, "alpha.md", marker + "\n")

        first = validate_repository(self.root)
        second = validate_repository(self.root)

        self.assertEqual(first, second)
        self.assertEqual(sorted(first), first)
        self.assertTrue(all(isinstance(item, Finding) for item in first))

    def test_cli_prints_findings_and_summary(self):
        marker = "TB" + "D"
        write_text(self.root, "notes.md", marker + "\n")
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            result = main(["--root", str(self.root)])

        self.assertEqual(1, result)
        self.assertIn("notes.md:1: PLACEHOLDER", output.getvalue())
        self.assertIn("Repository validation failed: 1 finding", output.getvalue())

    def test_cli_prints_success_summary(self):
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            result = main(["--root", str(self.root)])

        self.assertEqual(0, result)
        self.assertEqual("Repository validation passed: no findings.\n", output.getvalue())


if __name__ == "__main__":
    unittest.main()
