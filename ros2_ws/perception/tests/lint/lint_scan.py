#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: diaofanghaiyang@hachist.com
上海哈船智能船舶技术有限公司
File: lint_scan.py
Brief: A19-LINT-1 static scans over everything that is compiled into perception_node.

Description:
Three scans (19 section 14, A19-LINT-1; 19 section 10):

  clock      Every timestamp, timeout and age is CLOCK_MONOTONIC (std::chrono::steady_clock).
             system_clock, CLOCK_REALTIME, gettimeofday, time(NULL) and a bare rclcpp::Clock()
             are findings. Wall clock is allowed only where the contract asks for a wall-clock
             value (the envelope ts field) or for human-readable logging, and every such place
             carries the marker below.
  ros        P19-4: perception data is not published on ROS topics. Any ROS publisher creation
             is a finding. The ROS executor exists to subscribe (pose / TF) and to run the
             camera command service.
  endpoint   P19-5: no hardcoded transport endpoint in code. Endpoints come from the resolved
             configuration; a literal like "tcp/127.0.0.1:7447" is a finding.

What is scanned. The translation units listed in <sources.txt> -- the build writes that list
from the perception_node target itself, so a unit added to the service is scanned without
anybody remembering to list it -- and every project file they include with #include "...",
followed transitively (header-only code is compiled into the service too). Files of the SDKs
and of the system are not ours and are not followed.

An allowed wall-clock, publisher or endpoint site carries an explicit marker on the same line,
or anywhere in the contiguous comment block directly above it:

    // LINT-ALLOW(<scan>): <reason>

The report lists every allowance: the markers are part of the evidence.

Self-test (--self-test). A scan that finds nothing proves nothing unless it is shown to find a
violation. Two controls per scan: the rule on a synthetic line, with and without the marker;
and the contract's own mutants put into a COPY of a real translation unit and of a header it
includes (an image publisher, a hardcoded "tcp/..." literal, a system_clock read), scanned
through the same file discovery as the real run. The run fails when any control is missed.

Usage: lint_scan.py <repo> <sources.txt> <out.json> [--self-test]
Exit code 1 if any finding survives, a listed unit is missing, or a control is not caught.
A static scan is not a proof of runtime behaviour.
"""
import json, os, re, shutil, sys, tempfile

repo, sources_file, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
self_test = "--self-test" in sys.argv

MARKER = re.compile(r"//\s*LINT-ALLOW\((\w+)\):\s*(.+)")

RULES = {
    "clock": [
        (re.compile(r"\bsystem_clock\b"), "system_clock"),
        (re.compile(r"\bCLOCK_REALTIME\b"), "CLOCK_REALTIME"),
        (re.compile(r"\bgettimeofday\s*\("), "gettimeofday"),
        (re.compile(r"\btime\s*\(\s*(NULL|nullptr|0)\s*\)"), "time(NULL)"),
        (re.compile(r"\brclcpp::Clock\s*\(\s*\)"), "bare rclcpp::Clock()"),
    ],
    "ros": [
        (re.compile(r"\bcreate_publisher\b"), "ROS publisher creation"),
        (re.compile(r"\badvertise\s*<"), "ROS advertise"),
        (re.compile(r"\bimage_transport\b"), "image_transport publisher"),
    ],
    "endpoint": [
        (re.compile(r'"(tcp|udp|quic|serial)/[^"]*"'), "hardcoded transport endpoint"),
        (re.compile(r'"\d{1,3}(\.\d{1,3}){3}:\d+"'), "hardcoded ip:port"),
        (re.compile(r'"(tcp|udp)://[^"]*"'), "hardcoded url endpoint"),
    ],
}


def scan_text(text, path):
    findings, allowances = [], []
    lines = text.splitlines()
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("//") or stripped.startswith("*"):
            continue          # comments describe the rule, they do not break it
        for scan, rules in RULES.items():
            for pattern, what in rules:
                if not pattern.search(line):
                    continue
                allow = None
                probes = [line]
                j = i - 1                      # the comment block directly above
                while j >= 0 and (lines[j].strip().startswith("//") or lines[j].strip().startswith("*")):
                    probes.append(lines[j])
                    j -= 1
                for probe in probes:
                    m = MARKER.search(probe)
                    if m and m.group(1) == scan:
                        allow = m.group(2).strip()
                        break
                if allow:
                    allowances.append({"scan": scan, "file": path, "line": i + 1,
                                       "what": what, "reason": allow})
                else:
                    findings.append({"scan": scan, "file": path, "line": i + 1,
                                     "what": what, "text": stripped[:160]})
    return findings, allowances


INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')


def resolve(root, including, name):
    """A quoted include as a path inside the project, or None when it is not ours."""
    for base in (os.path.dirname(including), "include", "src", "tests"):
        candidate = os.path.normpath(os.path.join(base, name))
        if not candidate.startswith("..") and os.path.isfile(os.path.join(root, candidate)):
            return candidate
    return None


def scan_files(root, files):
    """The listed units and every project file they include, each once."""
    findings, allowances, scanned, missing = [], [], [], []
    queue = []
    for rel in files:
        rel = os.path.relpath(rel, root) if os.path.isabs(rel) else os.path.normpath(rel)
        if not os.path.isfile(os.path.join(root, rel)):
            missing.append(rel)
        elif rel not in queue:
            queue.append(rel)
    while queue:
        rel = queue.pop(0)
        if rel in scanned:
            continue
        scanned.append(rel)
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            text = f.read()
        f_findings, f_allow = scan_text(text, rel)
        findings += f_findings
        allowances += f_allow
        for line in text.splitlines():
            m = INCLUDE.match(line)
            if m:
                target = resolve(root, rel, m.group(1))
                if target and target not in scanned and target not in queue:
                    queue.append(target)
    return findings, allowances, scanned, missing


MUTANTS = {
    "clock": "    const auto now = std::chrono::system_clock::now();\n",
    "ros": "    auto pub = node->create_publisher<sensor_msgs::msg::Image>(\"perception/image\", 1);\n",
    "endpoint": "    const char* ep = \"tcp/127.0.0.1:7447\";\n",
}


def mutant_check():
    """Each scan must catch its own violation; a scan that catches nothing is
    not evidence that the code is clean."""
    results = {}
    for scan, injected in MUTANTS.items():
        text = "void f() {\n" + injected + "}\n"
        findings, _ = scan_text(text, "<mutant>")
        caught = [f for f in findings if f["scan"] == scan]
        # and the same line with an allowance marker must NOT be reported
        allowed_text = "void f() {\n" + injected.rstrip("\n") + "  // LINT-ALLOW(%s): mutant control\n}\n" % scan
        allowed_findings, allowed_marks = scan_text(allowed_text, "<mutant-allowed>")
        results[scan] = {"caught": len(caught), "caught_what": [f["what"] for f in caught],
                         "allowed_suppressed": len([f for f in allowed_findings if f["scan"] == scan]) == 0,
                         "allowance_recorded": len([a for a in allowed_marks if a["scan"] == scan]) == 1}
    return results


def injected_file_check(root, sources):
    """The contract's mutants in a copy of a real unit and of a header it includes."""
    unit = next((s for s in sources if os.path.isfile(os.path.join(root, s))), None)
    if unit is None:
        return {"ran": False}
    with open(os.path.join(root, unit), encoding="utf-8", errors="replace") as f:
        unit_text = f.read()
    header = next((resolve(root, unit, m.group(1)) for m in map(INCLUDE.match, unit_text.splitlines()) if m and resolve(root, unit, m.group(1))), None)
    results = {"ran": True, "unit": unit, "header": header}
    for scan, injected in MUTANTS.items():
        for where in ("unit", "header"):
            target = unit if where == "unit" else header
            if target is None:
                results["%s_in_%s" % (scan, where)] = False
                continue
            with tempfile.TemporaryDirectory() as tmp:
                for rel in {unit, header} - {None}:
                    os.makedirs(os.path.dirname(os.path.join(tmp, rel)), exist_ok=True)
                    shutil.copyfile(os.path.join(root, rel), os.path.join(tmp, rel))
                with open(os.path.join(tmp, target), "a", encoding="utf-8") as f:
                    f.write("\nvoid lint_scan_control() {\n" + injected + "}\n")
                found, _, _, _ = scan_files(tmp, [unit])
                results["%s_in_%s" % (scan, where)] = any(x["scan"] == scan and x["file"] == target for x in found)
    return results


with open(sources_file, encoding="utf-8") as f:
    sources = [line.strip() for line in f if line.strip()]
sources = [os.path.relpath(s, repo) if os.path.isabs(s) else os.path.normpath(s) for s in sources]
findings, allowances, scanned, missing = scan_files(repo, sources)
mutants = mutant_check() if self_test else {}
mutants_ok = all(m["caught"] >= 1 and m["allowed_suppressed"] and m["allowance_recorded"] for m in mutants.values())
injected = injected_file_check(repo, sources) if self_test else {"ran": False}
injected_ok = (not self_test) or (injected.get("ran", False) and all(v for k, v in injected.items() if "_in_" in k))

report = {
    "scope": "static scan of the translation units compiled into perception_node and of every project file "
             "they include, transitively. Not a proof of runtime behaviour.",
    "sources_list": sources_file,
    "translation_units": len(sources),
    "files_scanned": len(scanned),
    "missing_units": missing,
    "findings": findings,
    "finding_count": len(findings),
    "allowances": allowances,
    "allowance_count": len(allowances),
    "self_test": self_test,
    "mutant_controls": mutants,
    "mutant_controls_pass": mutants_ok,
    "injected_file_controls": injected,
    "injected_file_controls_pass": injected_ok,
    "result": "PASS" if not findings and not missing and len(sources) > 0 and mutants_ok and injected_ok else "FAIL",
}
with open(out_path, "w", encoding="utf-8") as f:
    json.dump(report, f, indent=1, ensure_ascii=False, sort_keys=True)
print(json.dumps({k: report[k] for k in ("result", "translation_units", "files_scanned", "finding_count", "allowance_count",
                                         "missing_units", "mutant_controls_pass", "injected_file_controls_pass")}, ensure_ascii=False))
for f in findings[:40]:
    print("  %s %s:%d %s | %s" % (f["scan"], f["file"], f["line"], f["what"], f["text"]))
sys.exit(0 if report["result"] == "PASS" else 1)
