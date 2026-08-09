#!/usr/bin/env python3
"""Validate the documentation-first AUTO_ROVER repository contract."""

import argparse
import json
import os
import re
from dataclasses import dataclass
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from urllib.parse import unquote, urlsplit


@dataclass(frozen=True, order=True)
class Finding:
    path: str
    line: int
    code: str
    message: str


@dataclass(frozen=True)
class YamlToken:
    line: int
    indent: int
    content: str


REQUIRED_PATHS = (
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

REQUIRED_POLICY = {
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

ROOT_EXCLUDED_PARTS = {
    ".catkin_tools",
    ".git",
    ".venv",
    "build",
    "devel",
    "env",
    "generated",
    "install",
    "log",
    "venv",
}

ANY_EXCLUDED_PARTS = {
    ".pytest_cache",
    "__pycache__",
    "htmlcov",
    "third_party",
}

TEXT_SUFFIXES = {
    ".action",
    ".cfg",
    ".cmake",
    ".conf",
    ".cpp",
    ".css",
    ".h",
    ".hpp",
    ".html",
    ".idl",
    ".ini",
    ".json",
    ".launch",
    ".md",
    ".msg",
    ".proto",
    ".ps1",
    ".py",
    ".repos",
    ".rosinstall",
    ".rst",
    ".rviz",
    ".sh",
    ".srv",
    ".toml",
    ".ts",
    ".txt",
    ".urdf",
    ".xml",
    ".xacro",
    ".yaml",
    ".yml",
}

TEXT_NAMES = {
    ".editorconfig",
    ".gitattributes",
    ".gitignore",
    "CMakeLists.txt",
    "Dockerfile",
    "LICENSE",
    "Makefile",
    "NOTICE",
}

MARKERS = ("TO" + "DO", "FIX" + "ME", "TB" + "D", "X" + "XX")
MARKER_PATTERN = re.compile(
    r"\b(?:" + "|".join(re.escape(item) for item in MARKERS) + r")\b",
    re.IGNORECASE,
)
FENCE_PATTERN = re.compile(r"^( {0,3})(`{3,}|~{3,})(.*)$")
TOP_LEVEL_KEY_PATTERN = re.compile(r"^([A-Za-z_][A-Za-z0-9_-]*):(?:\s|$)")

ACTION_PINS = {
    "actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1",
    "actions/setup-python@5fda3b95a4ea91299a34e894583c3862153e4b97",
}

UNIT_TEST_COMMAND = (
    'python -m unittest discover -s tests/repository -p "test_*.py" -v'
)
VALIDATOR_COMMAND = "python tools/ci/validate_repository.py --root ."


class DuplicateKeyError(ValueError):
    pass


class InvalidConstantError(ValueError):
    pass


def _relative(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def _is_excluded(relative: Path) -> bool:
    parts = relative.parts
    if len(parts) >= 3 and parts[:3] == ("docs", "superpowers", "plans"):
        return True
    if parts and parts[0] in ROOT_EXCLUDED_PARTS:
        return True
    return any(part in ANY_EXCLUDED_PARTS for part in parts)


def _iter_text_files(root: Path) -> Iterable[Path]:
    paths = []
    for directory, child_directories, filenames in os.walk(str(root)):
        current = Path(directory)
        current_relative = current.relative_to(root)
        child_directories[:] = sorted(
            name
            for name in child_directories
            if not _is_excluded(current_relative / name)
            and not (current / name).is_symlink()
        )
        for filename in sorted(filenames):
            path = current / filename
            relative = path.relative_to(root)
            if _is_excluded(relative) or path.is_symlink():
                continue
            if path.name in TEXT_NAMES or path.suffix.lower() in TEXT_SUFFIXES:
                paths.append(path)
    return sorted(paths, key=lambda item: _relative(item, root))


def _validate_tree_paths(root: Path) -> List[Finding]:
    """Reject ambiguous path spelling and links in first-party repository content."""

    findings = []
    root_resolved = root.resolve()
    for directory, child_directories, filenames in os.walk(
        str(root), followlinks=False
    ):
        current = Path(directory)
        current_relative = current.relative_to(root)
        included_names = [
            name
            for name in child_directories + filenames
            if not _is_excluded(current_relative / name)
        ]

        folded_names = {}
        for name in included_names:
            folded_names.setdefault(name.casefold(), []).append(name)
        for names in folded_names.values():
            distinct_names = sorted(set(names))
            if len(distinct_names) < 2:
                continue
            for name in distinct_names:
                relative = current_relative / name
                others = [item for item in distinct_names if item != name]
                findings.append(
                    Finding(
                        relative.as_posix(),
                        1,
                        "PATH_CASE_COLLISION",
                        "path collides by letter case with: " + ", ".join(others),
                    )
                )

        for name in included_names:
            path = current / name
            if not path.is_symlink():
                continue
            relative = path.relative_to(root)
            try:
                resolved = path.resolve(strict=False)
            except (OSError, RuntimeError):
                resolved = path.absolute()
            code = (
                "PATH_SYMLINK"
                if _inside_root(root_resolved, resolved)
                else "PATH_SYMLINK_ESCAPE"
            )
            message = (
                "first-party repository paths must not be symbolic links"
                if code == "PATH_SYMLINK"
                else "symbolic link resolves outside the repository"
            )
            findings.append(Finding(relative.as_posix(), 1, code, message))

        child_directories[:] = sorted(
            name
            for name in child_directories
            if not _is_excluded(current_relative / name)
            and not (current / name).is_symlink()
        )
    return findings


def _has_symlink_component(root: Path, relative: Path) -> bool:
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            return True
    return False


def _case_status(root: Path, relative: Path) -> Tuple[str, Optional[Path]]:
    current = root
    mismatch = False
    for part in relative.parts:
        if part in ("", "."):
            continue
        try:
            entries = list(current.iterdir())
        except (FileNotFoundError, NotADirectoryError, PermissionError):
            return "missing", None
        exact = next((entry for entry in entries if entry.name == part), None)
        if exact is not None:
            current = exact
            continue
        folded = [entry for entry in entries if entry.name.casefold() == part.casefold()]
        if len(folded) != 1:
            return "missing", None
        mismatch = True
        current = folded[0]
    return ("case" if mismatch else "exact"), current


def _validate_required(root: Path) -> List[Finding]:
    findings = []
    for relative_text in REQUIRED_PATHS:
        status, actual = _case_status(root, Path(relative_text))
        if status == "missing" or actual is None:
            findings.append(
                Finding(relative_text, 1, "REQUIRED_MISSING", "required file is missing")
            )
        elif status == "case":
            findings.append(
                Finding(
                    relative_text,
                    1,
                    "REQUIRED_CASE",
                    "required path exists with different letter case",
                )
            )
        elif _has_symlink_component(root, Path(relative_text)):
            # The tree audit reports the link itself. Do not follow it to inspect
            # required content, especially when it points outside the repository.
            continue
        elif not actual.is_file():
            findings.append(
                Finding(relative_text, 1, "REQUIRED_MISSING", "required path is not a file")
            )
        elif not actual.read_bytes().strip():
            findings.append(
                Finding(relative_text, 1, "REQUIRED_EMPTY", "required file is empty")
            )
    return findings


def _decode_text_files(root: Path) -> Tuple[Dict[Path, str], List[Finding]]:
    texts = {}
    findings = []
    for path in _iter_text_files(root):
        try:
            texts[path] = path.read_text(encoding="utf-8")
        except UnicodeDecodeError as error:
            line_number = path.read_bytes()[: error.start].count(b"\n") + 1
            findings.append(
                Finding(
                    _relative(path, root),
                    line_number,
                    "TEXT_ENCODING",
                    "first-party text is not valid UTF-8",
                )
            )
    return texts, findings


def _validate_markers(path: Path, root: Path, text: str) -> List[Finding]:
    findings = []
    for line_number, line in enumerate(text.splitlines(), 1):
        match = MARKER_PATTERN.search(line)
        if match:
            findings.append(
                Finding(
                    _relative(path, root),
                    line_number,
                    "PLACEHOLDER",
                    "prohibited unfinished-work marker: " + match.group(0),
                )
            )
    return findings


def _inside_root(root: Path, candidate: Path) -> bool:
    try:
        return os.path.commonpath((str(root), str(candidate))) == str(root)
    except ValueError:
        return False


def _link_target(raw_target: str) -> Optional[str]:
    target = raw_target.strip()
    if not target or target.startswith("#") or target.startswith("//"):
        return None
    local_candidate = unquote(target.split("#", 1)[0].split("?", 1)[0])
    if PurePosixPath(local_candidate).is_absolute() or PureWindowsPath(
        local_candidate
    ).is_absolute():
        return local_candidate
    parsed = urlsplit(target)
    if parsed.scheme == "file":
        return unquote(parsed.path)
    if parsed.scheme:
        return None
    return unquote(parsed.path)


def _validate_link(
    source: Path,
    root: Path,
    line_number: int,
    raw_target: str,
) -> Optional[Finding]:
    target = _link_target(raw_target)
    if target is None or target == "":
        return None
    target_path = Path(target)
    if (
        target_path.is_absolute()
        or PurePosixPath(target).is_absolute()
        or PureWindowsPath(target).is_absolute()
    ):
        return Finding(
            _relative(source, root),
            line_number,
            "LINK_ESCAPE",
            "local link is absolute or escapes the repository: " + target,
        )
    root_resolved = root.resolve()
    lexical_candidate = Path(os.path.abspath(str(source.parent / target_path)))
    resolved_candidate = lexical_candidate.resolve(strict=False)
    if not _inside_root(root_resolved, resolved_candidate):
        return Finding(
            _relative(source, root),
            line_number,
            "LINK_ESCAPE",
            "local link escapes the repository: " + target,
        )
    relative_target = Path(os.path.relpath(str(lexical_candidate), str(root_resolved)))
    status, _ = _case_status(root_resolved, relative_target)
    if status == "case":
        return Finding(
            _relative(source, root),
            line_number,
            "LINK_CASE",
            "local link has incorrect path case: " + target,
        )
    if status == "missing":
        return Finding(
            _relative(source, root),
            line_number,
            "LINK_MISSING",
            "local link target does not exist: " + target,
        )
    return None


def _blank_preserving_newlines(value: str) -> str:
    return "".join(character if character in "\r\n" else " " for character in value)


def _mask_fenced_code(
    path: Path, root: Path, text: str
) -> Tuple[str, List[Finding]]:
    visible_lines = []
    findings = []
    fence_character = None
    fence_length = 0
    opening_line = 0
    for line_number, line in enumerate(text.splitlines(keepends=True), 1):
        content = line.rstrip("\r\n")
        match = FENCE_PATTERN.match(content)
        if fence_character is None:
            if match:
                fence = match.group(2)
                if fence[0] == "`" and "`" in match.group(3):
                    findings.append(
                        Finding(
                            _relative(path, root),
                            line_number,
                            "MARKDOWN_FENCE",
                            "backtick fence info string must not contain backticks",
                        )
                    )
                    visible_lines.append(line)
                    continue
                fence_character = fence[0]
                fence_length = len(fence)
                opening_line = line_number
                visible_lines.append(_blank_preserving_newlines(line))
                continue
            visible_lines.append(line)
            continue

        visible_lines.append(_blank_preserving_newlines(line))
        if match:
            fence = match.group(2)
            remainder = match.group(3)
            if (
                fence[0] == fence_character
                and len(fence) >= fence_length
                and not remainder.strip()
            ):
                fence_character = None
                fence_length = 0
                opening_line = 0
    if fence_character is not None:
        findings.append(
            Finding(
                _relative(path, root),
                opening_line,
                "MARKDOWN_FENCE",
                "fenced code block is not closed",
            )
        )
    return "".join(visible_lines), findings


def _is_escaped(text: str, index: int) -> bool:
    backslashes = 0
    cursor = index - 1
    while cursor >= 0 and text[cursor] == "\\":
        backslashes += 1
        cursor -= 1
    return backslashes % 2 == 1


def _mask_inline_code(text: str) -> str:
    characters = list(text)
    cursor = 0
    while cursor < len(text):
        if text[cursor] != "`" or _is_escaped(text, cursor):
            cursor += 1
            continue
        run_end = cursor
        while run_end < len(text) and text[run_end] == "`":
            run_end += 1
        delimiter = text[cursor:run_end]
        search_from = run_end
        closing_start = -1
        while True:
            candidate = text.find(delimiter, search_from)
            if candidate < 0:
                break
            before_matches = candidate > 0 and text[candidate - 1] == "`"
            after = candidate + len(delimiter)
            after_matches = after < len(text) and text[after] == "`"
            if not before_matches and not after_matches:
                closing_start = candidate
                break
            search_from = candidate + 1
        if closing_start < 0:
            cursor = run_end
            continue
        closing_end = closing_start + len(delimiter)
        for index in range(cursor, closing_end):
            if characters[index] not in "\r\n":
                characters[index] = " "
        cursor = closing_end
    return "".join(characters)


def _skip_markdown_space(text: str, cursor: int, allow_newline: bool) -> int:
    while cursor < len(text) and text[cursor] in " \t":
        cursor += 1
    if allow_newline and cursor < len(text) and text[cursor] in "\r\n":
        if text[cursor] == "\r" and cursor + 1 < len(text) and text[cursor + 1] == "\n":
            cursor += 2
        else:
            cursor += 1
        while cursor < len(text) and text[cursor] in " \t":
            cursor += 1
    return cursor


def _unescape_markdown(value: str) -> str:
    return re.sub(r"\\([!\"#$%&'()*+,\-./:;<=>?@\[\\\]^_`{|}~])", r"\1", value)


def _parse_markdown_destination(
    text: str, cursor: int, allow_newline: bool
) -> Tuple[Optional[str], int]:
    cursor = _skip_markdown_space(text, cursor, allow_newline)
    if cursor >= len(text):
        return None, cursor

    if text[cursor] == "<":
        start = cursor + 1
        cursor = start
        while cursor < len(text):
            if text[cursor] == ">" and not _is_escaped(text, cursor):
                return _unescape_markdown(text[start:cursor]), cursor + 1
            if text[cursor] in "\r\n":
                return None, cursor
            cursor += 1
        return None, cursor

    start = cursor
    depth = 0
    while cursor < len(text):
        character = text[cursor]
        if character == "\\" and cursor + 1 < len(text):
            cursor += 2
            continue
        if character == "(":
            depth += 1
            cursor += 1
            continue
        if character == ")":
            if depth == 0:
                break
            depth -= 1
            cursor += 1
            continue
        if character.isspace() and depth == 0:
            break
        cursor += 1
    if cursor == start or depth != 0:
        return None, cursor
    return _unescape_markdown(text[start:cursor]), cursor


def _find_inline_link_close(text: str, cursor: int) -> Optional[int]:
    cursor = _skip_markdown_space(text, cursor, True)
    if cursor >= len(text):
        return None
    if text[cursor] == ")":
        return cursor

    if text[cursor] in "\"'":
        delimiter = text[cursor]
        cursor += 1
        while cursor < len(text):
            if text[cursor] == "\\" and cursor + 1 < len(text):
                cursor += 2
                continue
            if text[cursor] == delimiter:
                cursor += 1
                break
            if text[cursor] in "\r\n":
                return None
            cursor += 1
        else:
            return None
    elif text[cursor] == "(":
        depth = 1
        cursor += 1
        while cursor < len(text) and depth:
            if text[cursor] == "\\" and cursor + 1 < len(text):
                cursor += 2
                continue
            if text[cursor] == "(":
                depth += 1
            elif text[cursor] == ")":
                depth -= 1
            if text[cursor] in "\r\n":
                return None
            cursor += 1
        if depth:
            return None
    else:
        return None

    cursor = _skip_markdown_space(text, cursor, True)
    if cursor < len(text) and text[cursor] == ")":
        return cursor
    return None


def _iter_inline_markdown_links(text: str) -> Iterable[Tuple[int, str]]:
    label_stack = []
    cursor = 0
    while cursor < len(text):
        character = text[cursor]
        if character == "[" and not _is_escaped(text, cursor):
            label_stack.append(cursor)
            cursor += 1
            continue
        if character != "]" or _is_escaped(text, cursor) or not label_stack:
            cursor += 1
            continue
        label_stack.pop()
        opening_destination = _skip_markdown_space(text, cursor + 1, True)
        if opening_destination >= len(text) or text[opening_destination] != "(":
            cursor += 1
            continue
        target, after_target = _parse_markdown_destination(
            text, opening_destination + 1, True
        )
        if target is None:
            cursor = opening_destination + 1
            continue
        close = _find_inline_link_close(text, after_target)
        if close is None:
            cursor = opening_destination + 1
            continue
        yield opening_destination, target
        label_stack = []
        cursor = close + 1


def _iter_reference_destinations(text: str) -> Iterable[Tuple[int, str]]:
    definition = re.compile(r"(?m)^ {0,3}\[(?!\^)[^\]\r\n]+\]:[ \t]*")
    for match in definition.finditer(text):
        target, _ = _parse_markdown_destination(text, match.end(), True)
        if target is not None:
            yield match.start(), target


def _validate_markdown(path: Path, root: Path, text: str) -> List[Finding]:
    visible, findings = _mask_fenced_code(path, root, text)
    visible = _mask_inline_code(visible)
    destinations = list(_iter_reference_destinations(visible))
    destinations.extend(_iter_inline_markdown_links(visible))
    for position, target in destinations:
        line_number = visible.count("\n", 0, position) + 1
        finding = _validate_link(path, root, line_number, target)
        if finding is not None:
            findings.append(finding)
    return findings


def _reject_duplicate_keys(pairs: Sequence[Tuple[str, object]]) -> Dict[str, object]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise DuplicateKeyError(key)
        result[key] = value
    return result


def _reject_non_standard_constant(value: str) -> object:
    raise InvalidConstantError(value)


def _load_strict_json(path: Path, root: Path, text: str) -> Tuple[Optional[object], List[Finding]]:
    relative = _relative(path, root)
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_non_standard_constant,
        )
    except DuplicateKeyError as error:
        return None, [
            Finding(relative, 1, "JSON_DUPLICATE_KEY", "duplicate JSON key: " + str(error))
        ]
    except InvalidConstantError as error:
        return None, [
            Finding(
                relative,
                1,
                "JSON_INVALID",
                "non-standard JSON constant: " + str(error),
            )
        ]
    except json.JSONDecodeError as error:
        return None, [
            Finding(relative, error.lineno, "JSON_INVALID", "invalid JSON: " + error.msg)
        ]
    return value, []


def _validate_policy_subset(
    actual: object,
    expected: object,
    prefix: str,
    output: List[Tuple[str, object, object]],
) -> None:
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            output.append((prefix, expected, actual))
            return
        for key, expected_value in expected.items():
            child = key if not prefix else prefix + "." + key
            if key not in actual:
                output.append((child, expected_value, "<missing>"))
            else:
                _validate_policy_subset(actual[key], expected_value, child, output)
        return
    if type(actual) is not type(expected) or actual != expected:
        output.append((prefix, expected, actual))


def _validate_architecture_policy(value: object) -> List[Finding]:
    mismatches = []
    _validate_policy_subset(value, REQUIRED_POLICY, "", mismatches)
    return [
        Finding(
            "docs/architecture/architecture-policy.json",
            1,
            "POLICY_VALUE",
            "required architecture policy value differs at " + key,
        )
        for key, _, _ in mismatches
    ]


def _yaml_basics(path: Path, root: Path, text: str) -> Tuple[Dict[str, int], List[Finding]]:
    relative = _relative(path, root)
    findings = []
    top_level = {}
    for line_number, line in enumerate(text.splitlines(), 1):
        if "\t" in line:
            findings.append(
                Finding(relative, line_number, "YAML_TAB", "YAML must not contain tabs")
            )
        stripped = line.lstrip(" ")
        indentation = len(line) - len(stripped)
        if stripped and not stripped.startswith("#") and indentation % 2:
            findings.append(
                Finding(
                    relative,
                    line_number,
                    "YAML_INDENT",
                    "YAML indentation must use multiples of two spaces",
                )
            )
        if indentation == 0:
            match = TOP_LEVEL_KEY_PATTERN.match(line)
            if match:
                key = match.group(1)
                if key in top_level:
                    findings.append(
                        Finding(
                            relative,
                            line_number,
                            "YAML_DUPLICATE_KEY",
                            "duplicate top-level YAML key: " + key,
                        )
                    )
                else:
                    top_level[key] = line_number
    return top_level, findings


def _validate_issue_form(path: Path, root: Path, text: str) -> List[Finding]:
    relative = _relative(path, root)
    top_level, findings = _yaml_basics(path, root, text)
    for key in ("name", "description", "body"):
        if key not in top_level:
            findings.append(
                Finding(relative, 1, "ISSUE_FORM_KEY", "missing Issue Form key: " + key)
            )
    ids = {}
    for line_number, line in enumerate(text.splitlines(), 1):
        match = re.match(r"^\s+id:\s*([^\s#]+)", line)
        if not match:
            continue
        identifier = match.group(1)
        if not re.match(r"^[A-Za-z0-9_-]+$", identifier):
            findings.append(
                Finding(
                    relative,
                    line_number,
                    "ISSUE_FORM_ID",
                    "Issue Form id contains unsupported characters: " + identifier,
                )
            )
        elif identifier in ids:
            findings.append(
                Finding(
                    relative,
                    line_number,
                    "ISSUE_FORM_DUPLICATE_ID",
                    "duplicate Issue Form body id: " + identifier,
                )
            )
        else:
            ids[identifier] = line_number
    return findings


def _validate_issue_config(path: Path, root: Path, text: str) -> List[Finding]:
    relative = _relative(path, root)
    top_level, findings = _yaml_basics(path, root, text)
    for key in ("blank_issues_enabled", "contact_links"):
        if key not in top_level:
            findings.append(
                Finding(relative, 1, "ISSUE_CONFIG_KEY", "missing issue config key: " + key)
            )
    return findings


def _strip_yaml_comment(line: str) -> str:
    quote = None
    cursor = 0
    while cursor < len(line):
        character = line[cursor]
        if quote == '"':
            if character == "\\" and cursor + 1 < len(line):
                cursor += 2
                continue
            if character == '"':
                quote = None
            cursor += 1
            continue
        if quote == "'":
            if character == "'" and cursor + 1 < len(line) and line[cursor + 1] == "'":
                cursor += 2
                continue
            if character == "'":
                quote = None
            cursor += 1
            continue
        if character in "\"'":
            quote = character
            cursor += 1
            continue
        if character == "#" and (cursor == 0 or line[cursor - 1].isspace()):
            return line[:cursor].rstrip()
        cursor += 1
    return line.rstrip()


def _yaml_tokens(text: str) -> List[YamlToken]:
    tokens = []
    block_scalar_indent = None
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        if not raw_line.strip():
            continue
        indentation = len(raw_line) - len(raw_line.lstrip(" "))
        if block_scalar_indent is not None:
            if indentation > block_scalar_indent:
                continue
            block_scalar_indent = None
        content = _strip_yaml_comment(raw_line[indentation:])
        if not content:
            continue
        token = YamlToken(line_number, indentation, content)
        tokens.append(token)
        if re.search(r":\s*[|>][0-9+-]*\s*$", content):
            block_scalar_indent = indentation
    return tokens


def _mapping_pair(content: str) -> Optional[Tuple[str, str]]:
    candidate = content[2:] if content.startswith("- ") else content
    match = re.match(r"^([A-Za-z_][A-Za-z0-9_-]*):(?:\s*(.*))?$", candidate)
    if not match:
        match = re.match(
            r"^([\"'])([A-Za-z_][A-Za-z0-9_-]*)\1:(?:\s*(.*))?$",
            candidate,
        )
        if match:
            return match.group(2), (match.group(3) or "")
    if not match:
        return None
    return match.group(1), (match.group(2) or "")


def _yaml_scalar(value: str) -> str:
    stripped = value.strip()
    if (
        len(stripped) >= 2
        and stripped[0] == stripped[-1]
        and stripped[0] in "\"'"
    ):
        return stripped[1:-1]
    return stripped


def _block_end(tokens: Sequence[YamlToken], parent_index: int) -> int:
    parent_indent = tokens[parent_index].indent
    cursor = parent_index + 1
    while cursor < len(tokens) and tokens[cursor].indent > parent_indent:
        cursor += 1
    return cursor


def _direct_mapping(
    tokens: Sequence[YamlToken], parent_index: int, duplicates=None
) -> Dict[str, Tuple[int, str]]:
    expected_indent = tokens[parent_index].indent + 2
    output = {}
    for index in range(parent_index + 1, _block_end(tokens, parent_index)):
        token = tokens[index]
        if token.indent != expected_indent or token.content.startswith("- "):
            continue
        pair = _mapping_pair(token.content)
        if pair is not None:
            if pair[0] in output and duplicates is not None:
                duplicates.add((token.line, pair[0]))
            output[pair[0]] = (index, _yaml_scalar(pair[1]))
    return output


def _top_level_mapping(
    tokens: Sequence[YamlToken], duplicates=None
) -> Dict[str, Tuple[int, str]]:
    output = {}
    for index, token in enumerate(tokens):
        if token.indent != 0 or token.content.startswith("- "):
            continue
        pair = _mapping_pair(token.content)
        if pair is not None:
            if pair[0] in output and duplicates is not None:
                duplicates.add((token.line, pair[0]))
            output[pair[0]] = (index, _yaml_scalar(pair[1]))
    return output


def _list_values(tokens: Sequence[YamlToken], parent_index: int) -> List[str]:
    expected_indent = tokens[parent_index].indent + 2
    output = []
    for index in range(parent_index + 1, _block_end(tokens, parent_index)):
        token = tokens[index]
        if token.indent == expected_indent and token.content.startswith("- "):
            output.append(_yaml_scalar(token.content[2:]))
    return output


def _workflow_steps(
    tokens: Sequence[YamlToken], steps_index: int, duplicates=None
) -> List[Dict[str, Tuple[int, str]]]:
    item_indent = tokens[steps_index].indent + 2
    end = _block_end(tokens, steps_index)
    starts = [
        index
        for index in range(steps_index + 1, end)
        if tokens[index].indent == item_indent and tokens[index].content.startswith("- ")
    ]
    output = []
    for position, start in enumerate(starts):
        stop = starts[position + 1] if position + 1 < len(starts) else end
        values = {}
        first_pair = _mapping_pair(tokens[start].content)
        if first_pair is not None:
            values[first_pair[0]] = (start, _yaml_scalar(first_pair[1]))
        for index in range(start + 1, stop):
            token = tokens[index]
            if token.indent != item_indent + 2 or token.content.startswith("- "):
                continue
            pair = _mapping_pair(token.content)
            if pair is not None:
                if pair[0] in values and duplicates is not None:
                    duplicates.add((token.line, pair[0]))
                values[pair[0]] = (index, _yaml_scalar(pair[1]))
        output.append(values)
    return output


def _append_workflow_finding(
    findings: List[Finding], relative: str, code: str, message: str
) -> None:
    finding = Finding(relative, 1, code, message)
    if finding not in findings:
        findings.append(finding)


def _validate_workflow(path: Path, root: Path, text: str) -> List[Finding]:
    relative = _relative(path, root)
    _, findings = _yaml_basics(path, root, text)
    tokens = _yaml_tokens(text)
    duplicate_keys = set()
    top_level = _top_level_mapping(tokens, duplicate_keys)
    for token in tokens:
        if _mapping_pair(token.content) is None and not token.content.startswith("- "):
            _append_workflow_finding(
                findings,
                relative,
                "WORKFLOW_SYNTAX",
                "unsupported workflow syntax at line {}".format(token.line),
            )
    expected_top_level = {"name", "on", "permissions", "concurrency", "jobs"}
    for key in sorted(expected_top_level - set(top_level)):
        _append_workflow_finding(
            findings, relative, "WORKFLOW_KEY", "missing workflow key: " + key
        )
    for key in sorted(set(top_level) - expected_top_level):
        _append_workflow_finding(
            findings, relative, "WORKFLOW_KEY", "unexpected workflow key: " + key
        )
    if top_level.get("name", (-1, ""))[1] != "Repository validation":
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_KEY",
            "workflow name must be Repository validation",
        )

    event_entries = {}
    on_entry = top_level.get("on")
    if on_entry is not None and on_entry[1] == "":
        event_entries = _direct_mapping(tokens, on_entry[0], duplicate_keys)
    all_keys = [
        pair[0]
        for token in tokens
        for pair in [_mapping_pair(token.content)]
        if pair is not None
    ]
    if "pull_request_target" in all_keys:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_UNSAFE_EVENT",
            "pull_request_target is not allowed",
        )
    if set(event_entries) != {"pull_request", "push"}:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_EVENT",
            "workflow must run only on pull requests and pushes to main",
        )
    pull_request_entry = event_entries.get("pull_request")
    pull_request_unconditional = (
        pull_request_entry is not None
        and pull_request_entry[1] == ""
        and _block_end(tokens, pull_request_entry[0]) == pull_request_entry[0] + 1
    )
    if not pull_request_unconditional:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_EVENT",
            "pull_request trigger must be unconditional",
        )
    push_entry = event_entries.get("push")
    push_is_main_only = False
    if push_entry is not None and push_entry[1] == "":
        push_mapping = _direct_mapping(tokens, push_entry[0], duplicate_keys)
        branches = push_mapping.get("branches")
        if branches is not None and branches[1] == "":
            push_is_main_only = (
                set(push_mapping) == {"branches"}
                and _list_values(tokens, branches[0]) == ["main"]
            )
    if not push_is_main_only:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_EVENT",
            "push trigger must target main only",
        )

    permissions_valid = False
    permissions_entry = top_level.get("permissions")
    if permissions_entry is not None and permissions_entry[1] == "":
        permission_mapping = _direct_mapping(
            tokens, permissions_entry[0], duplicate_keys
        )
        permissions_valid = (
            set(permission_mapping) == {"contents"}
            and permission_mapping["contents"][1] == "read"
        )
    nested_permissions = any(
        token.indent > 0
        and (pair := _mapping_pair(token.content)) is not None
        and pair[0] == "permissions"
        for token in tokens
    )
    if not permissions_valid or nested_permissions:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_PERMISSIONS",
            "workflow permissions must be top-level contents read-only with no overrides",
        )

    concurrency_valid = False
    concurrency_entry = top_level.get("concurrency")
    if concurrency_entry is not None and concurrency_entry[1] == "":
        concurrency_mapping = _direct_mapping(
            tokens, concurrency_entry[0], duplicate_keys
        )
        concurrency_valid = (
            set(concurrency_mapping) == {"group", "cancel-in-progress"}
            and concurrency_mapping["group"][1]
            == "repository-validation-${{ github.ref }}"
            and concurrency_mapping["cancel-in-progress"][1] == "true"
        )
    if not concurrency_valid:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_CONCURRENCY",
            "workflow concurrency must group by github.ref and cancel superseded runs",
        )

    jobs_entry = top_level.get("jobs")
    jobs_mapping = {}
    if jobs_entry is not None and jobs_entry[1] == "":
        jobs_mapping = _direct_mapping(tokens, jobs_entry[0], duplicate_keys)
    required_entry = jobs_mapping.get("required")
    job_mapping = {}
    if required_entry is not None and required_entry[1] == "":
        job_mapping = _direct_mapping(tokens, required_entry[0], duplicate_keys)
    required_job_fields = {"name", "runs-on", "timeout-minutes", "steps"}
    if set(jobs_mapping) != {"required"} or set(job_mapping) != required_job_fields:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_JOB",
            "workflow must contain only the documented required job shape",
        )
    expected_job_values = (
        ("name", "required", "stable check name required"),
        ("runs-on", "ubuntu-24.04", "ubuntu-24.04 runner"),
        ("timeout-minutes", "5", "five-minute timeout"),
    )
    for key, expected, description in expected_job_values:
        if job_mapping.get(key, (-1, ""))[1] != expected:
            _append_workflow_finding(
                findings,
                relative,
                "WORKFLOW_JOB",
                "required job is missing " + description,
            )

    steps = []
    steps_entry = job_mapping.get("steps")
    if steps_entry is not None and steps_entry[1] == "":
        steps = _workflow_steps(tokens, steps_entry[0], duplicate_keys)
    expected_step_keys = (
        {"name", "uses"},
        {"name", "uses", "with"},
        {"name", "run"},
        {"name", "run"},
    )
    if len(steps) != 4 or any(
        set(step) != expected_step_keys[index]
        for index, step in enumerate(steps[:4])
    ):
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_JOB",
            "required job must contain exactly the four documented steps",
        )

    runs = [step["run"][1] for step in steps if "run" in step]
    for command in (UNIT_TEST_COMMAND, VALIDATOR_COMMAND):
        if command not in runs:
            _append_workflow_finding(
                findings,
                relative,
                "WORKFLOW_JOB",
                "required job is missing command: " + command,
            )

    python_version_valid = False
    for step in steps:
        uses = step.get("uses", (-1, ""))[1]
        with_entry = step.get("with")
        if uses.endswith("setup-python@5fda3b95a4ea91299a34e894583c3862153e4b97") and with_entry:
            with_mapping = _direct_mapping(tokens, with_entry[0], duplicate_keys)
            python_version_valid = (
                set(with_mapping) == {"python-version"}
                and with_mapping["python-version"][1] == "3.8.18"
            )
    if not python_version_valid:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_JOB",
            "required job is missing Python 3.8.18",
        )

    uses_entries = []
    for token in tokens:
        pair = _mapping_pair(token.content)
        if pair is not None and pair[0] == "uses":
            uses_entries.append(_yaml_scalar(pair[1]))
    invalid_actions = [action for action in uses_entries if action not in ACTION_PINS]
    missing_actions = sorted(ACTION_PINS - set(uses_entries))
    for action in invalid_actions:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_ACTION_PIN",
            "action is not an approved full-SHA pin: " + action,
        )
    for action in missing_actions:
        _append_workflow_finding(
            findings,
            relative,
            "WORKFLOW_ACTION_PIN",
            "required action is missing: " + action,
        )
    for line_number, key in sorted(duplicate_keys):
        findings.append(
            Finding(
                relative,
                line_number,
                "YAML_DUPLICATE_KEY",
                "duplicate workflow mapping key: " + key,
            )
        )
    return findings


def validate_repository(root: Path) -> List[Finding]:
    """Return deterministic repository-policy findings."""

    root = Path(root).resolve()
    findings = _validate_tree_paths(root)
    findings.extend(_validate_required(root))
    texts, encoding_findings = _decode_text_files(root)
    findings.extend(encoding_findings)

    architecture_policy_path = root / "docs/architecture/architecture-policy.json"
    for path, text in texts.items():
        findings.extend(_validate_markers(path, root, text))
        suffix = path.suffix.lower()
        if suffix == ".md":
            findings.extend(_validate_markdown(path, root, text))
        if suffix == ".json":
            value, json_findings = _load_strict_json(path, root, text)
            findings.extend(json_findings)
            if path == architecture_policy_path and value is not None:
                findings.extend(_validate_architecture_policy(value))

    issue_directory = root / ".github/ISSUE_TEMPLATE"
    for filename in ("bug-report.yml", "change-proposal.yml"):
        path = issue_directory / filename
        if path in texts:
            findings.extend(_validate_issue_form(path, root, texts[path]))
    config_path = issue_directory / "config.yml"
    if config_path in texts:
        findings.extend(_validate_issue_config(config_path, root, texts[config_path]))
    workflow_path = root / ".github/workflows/repository-validation.yml"
    if workflow_path in texts:
        findings.extend(_validate_workflow(workflow_path, root, texts[workflow_path]))

    return sorted(findings)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Print findings and return 0 for success or 1 for violations."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".", help="repository root to validate")
    arguments = parser.parse_args(argv)
    findings = validate_repository(Path(arguments.root))
    if findings:
        for finding in findings:
            print(
                "{}:{}: {} {}".format(
                    finding.path,
                    finding.line,
                    finding.code,
                    finding.message,
                )
            )
        noun = "finding" if len(findings) == 1 else "findings"
        print("Repository validation failed: {} {}.".format(len(findings), noun))
        return 1
    print("Repository validation passed: no findings.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
