#!/usr/bin/env python3
"""Report what this host has for the cuVSLAM + DepthAI stack. The exit code is always 0.

Runs unchanged on the Windows laptop, on WSL2 Ubuntu 22.04 and on a Jetson Orin Nano
(JetPack 6.x). It needs nothing beyond the standard library; every package it reports on is
imported inside a try/except, so a missing package is a line in the report, not a crash.
Nothing here opens the OAK camera or touches the GPU unless ``--warm-up`` is given.

Usage:
    python methods/cuvslam-vio/tools/check_env.py             human-readable report (one "key: value" line each)
    python methods/cuvslam-vio/tools/check_env.py --json      the same data as one JSON object on stdout
    python methods/cuvslam-vio/tools/check_env.py --warm-up   also call cuvslam.warm_up_gpu() (initialises CUDA,
                                          cuBLAS and cuSOLVER; raises if any of them fails)

What is reported:
    os          platform, distribution, WSL / Jetson detection, JetPack and L4T version
    python      version, executable, whether it runs inside a venv, machine (x86_64 / aarch64)
    nvidia-smi  present or not, driver version, the "CUDA Version" the driver supports, GPU name
    cuda        toolkit version from `nvcc --version` or /usr/local/cuda/version.json (or
                version.txt), the /usr/local/cuda-* directories, where libcuda.so.1 comes from
    cuvslam     `import cuvslam` works or not, get_version() and __version__
    depthai     `import depthai` works or not, its version
    packages    numpy, opencv (cv2), scipy, pyyaml, pillow, rerun-sdk, evo versions

Expected on the three hosts (see methods/cuvslam-vio/docs/cuvslam-setup.md, section 4):
    Windows laptop   nvidia-smi yes (driver 581.95), nvcc no, cuvslam NOT importable (no
                     Windows wheel exists; expected), depthai 3.10.0
    WSL2 Ubuntu      nvidia-smi yes (same Windows driver, from /usr/lib/wsl/lib), nvcc 12.6,
                     libcuda.so.1 from /usr/lib/wsl/lib, cuvslam 17.0.0+cu12
    Jetson Orin Nano nvidia-smi absent (integrated GPU; normal), nvcc 12.6 from
                     /usr/local/cuda-12.6, JetPack 6.2.x, cuvslam 17.0.0+cu12
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

Report = Dict[str, Any]


# ----------------------------------------------------------------------------- helpers

def run(cmd: List[str], timeout: float = 20.0) -> Tuple[int, str]:
    """Run a command and return (returncode, stdout+stderr). Never raises."""
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                              errors="replace")
        return proc.returncode, (proc.stdout or "") + (proc.stderr or "")
    except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
        return -1, f"{type(exc).__name__}: {exc}"


def read_text(path: Path, limit: int = 4096) -> Optional[str]:
    try:
        return path.read_text(encoding="utf-8", errors="replace")[:limit]
    except OSError:
        return None


def os_release() -> Dict[str, str]:
    """Parse /etc/os-release into a dict (empty on Windows)."""
    text = read_text(Path("/etc/os-release"))
    result: Dict[str, str] = {}
    if not text:
        return result
    for line in text.splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            result[key.strip()] = value.strip().strip('"')
    return result


def first_line(text: str) -> str:
    return text.strip().splitlines()[0] if text.strip() else ""


def module_version(name: str, attr: str = "__version__", dist: Optional[str] = None) -> Report:
    """Import a module and read its version; report the ImportError instead of raising."""
    info: Report = {"importable": False, "version": None, "error": None, "file": None}
    try:
        module = importlib.import_module(name)
    except Exception as exc:  # noqa: BLE001 - any failure is a report line, not a crash
        info["error"] = f"{type(exc).__name__}: {first_line(str(exc)) or exc}"
        return info
    info["importable"] = True
    info["file"] = getattr(module, "__file__", None)
    version = getattr(module, attr, None)
    if version is None and dist:
        try:
            from importlib import metadata
            version = metadata.version(dist)
        except Exception:  # noqa: BLE001
            version = None
    info["version"] = str(version) if version is not None else None
    return info


# ----------------------------------------------------------------------------- sections

def section_os() -> Report:
    info: Report = {
        "system": platform.system(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "distribution": None,
        "is_wsl": False,
        "wsl_distro": os.environ.get("WSL_DISTRO_NAME"),
        "is_jetson": False,
        "jetson_model": None,
        "l4t_release": None,
        "jetpack": None,
    }
    rel = os_release()
    if rel:
        info["distribution"] = rel.get("PRETTY_NAME") or rel.get("NAME")
    proc_version = read_text(Path("/proc/version")) or ""
    # Microsoft's WSL2 kernel identifies itself in /proc/version ("...-microsoft-standard-WSL2").
    if "microsoft" in proc_version.lower() or info["wsl_distro"]:
        info["is_wsl"] = True
    tegra = read_text(Path("/etc/nv_tegra_release"))
    if tegra:
        # First line looks like: "# R36 (release), REVISION: 4.4, GCID: ..., BOARD: generic, ..."
        info["is_jetson"] = True
        match = re.search(r"R(\d+)\s*\(release\),\s*REVISION:\s*([\d.]+)", tegra)
        info["l4t_release"] = f"{match.group(1)}.{match.group(2)}" if match else first_line(tegra)
    model = read_text(Path("/proc/device-tree/model"))
    if model:
        info["jetson_model"] = model.strip("\x00 \n")
        if "jetson" in model.lower():
            info["is_jetson"] = True
    if info["is_jetson"] and shutil.which("dpkg-query"):
        code, out = run(["dpkg-query", "-W", "-f=${Version}", "nvidia-jetpack"])
        if code == 0 and out.strip():
            info["jetpack"] = out.strip()
    return info


def parse_driver_cuda(smi_text: str) -> Optional[str]:
    match = re.search(r"CUDA Version:\s*([\d.]+)", smi_text)
    return match.group(1) if match else None


def section_nvidia_smi(is_wsl: bool) -> Report:
    info: Report = {"present": False, "path": None, "driver_version": None,
                    "driver_cuda_version": None, "gpus": [], "error": None}
    path = shutil.which("nvidia-smi")
    # In WSL2 the Windows driver's tools are mapped in under /usr/lib/wsl/lib; a root shell may
    # not have it on PATH (NVIDIA CUDA on WSL User Guide).
    if path is None and is_wsl:
        candidate = Path("/usr/lib/wsl/lib/nvidia-smi")
        if candidate.exists():
            path = str(candidate)
    if path is None:
        return info
    info["present"] = True
    info["path"] = path
    code, out = run([path, "--query-gpu=name,driver_version,compute_cap", "--format=csv,noheader"])
    if code == 0:
        for line in out.strip().splitlines():
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 2:
                info["gpus"].append({"name": parts[0], "driver_version": parts[1],
                                     "compute_cap": parts[2] if len(parts) > 2 else None})
        if info["gpus"]:
            info["driver_version"] = info["gpus"][0]["driver_version"]
    else:
        info["error"] = first_line(out)
    code2, out2 = run([path])
    if code2 == 0:
        info["driver_cuda_version"] = parse_driver_cuda(out2)
        if info["driver_version"] is None:
            match = re.search(r"Driver Version:\s*([\d.]+)", out2)
            info["driver_version"] = match.group(1) if match else None
    elif info["error"] is None:
        info["error"] = first_line(out2)
    return info


def find_nvcc() -> Optional[str]:
    path = shutil.which("nvcc")
    if path:
        return path
    for candidate in (Path("/usr/local/cuda/bin/nvcc"),):
        if candidate.exists():
            return str(candidate)
    cuda_path = os.environ.get("CUDA_PATH") or os.environ.get("CUDA_HOME")
    if cuda_path:
        for name in ("nvcc", "nvcc.exe"):
            candidate = Path(cuda_path) / "bin" / name
            if candidate.exists():
                return str(candidate)
    return None


def section_cuda() -> Report:
    info: Report = {"nvcc": None, "nvcc_version": None, "version_json": None,
                    "version_json_version": None, "version_txt_version": None,
                    "cuda_dirs": [], "cuda_symlink": None, "libcuda": None, "libcublas": None}
    nvcc = find_nvcc()
    if nvcc:
        info["nvcc"] = nvcc
        code, out = run([nvcc, "--version"])
        if code == 0:
            # "Cuda compilation tools, release 12.6, V12.6.77"
            match = re.search(r"release\s+([\d.]+),\s*V([\d.]+)", out)
            info["nvcc_version"] = match.group(2) if match else first_line(out)
    cuda_root = Path("/usr/local/cuda")
    if cuda_root.is_symlink():
        try:
            info["cuda_symlink"] = str(cuda_root.resolve())
        except OSError:
            pass
    version_json = cuda_root / "version.json"
    text = read_text(version_json)
    if text:
        info["version_json"] = str(version_json)
        try:
            data = json.loads(text)
            info["version_json_version"] = (data.get("cuda") or {}).get("version")
        except (ValueError, AttributeError):
            info["version_json_version"] = None
    text_txt = read_text(cuda_root / "version.txt")
    if text_txt:
        # Older layout: "CUDA Version 11.x.y"
        match = re.search(r"CUDA Version\s+([\d.]+)", text_txt)
        info["version_txt_version"] = match.group(1) if match else first_line(text_txt)
    try:
        info["cuda_dirs"] = sorted(str(p) for p in Path("/usr/local").glob("cuda-*") if p.is_dir())
    except OSError:
        pass
    if platform.system() == "Windows":
        cuda_path = os.environ.get("CUDA_PATH")
        if cuda_path:
            info["cuda_dirs"] = [cuda_path]
    # Where the driver library comes from: in WSL2 it is the stub NVIDIA maps in under
    # /usr/lib/wsl/lib; on a bare-metal Linux box it comes from the Linux driver package.
    wsl_libcuda = Path("/usr/lib/wsl/lib/libcuda.so.1")
    if wsl_libcuda.exists():
        info["libcuda"] = str(wsl_libcuda)
    elif shutil.which("ldconfig"):
        code, out = run(["ldconfig", "-p"])
        if code == 0:
            for line in out.splitlines():
                if "libcuda.so.1 " in line and "=>" in line:
                    info["libcuda"] = line.split("=>", 1)[1].strip()
                    break
            for line in out.splitlines():
                if "libcublas.so.12 " in line and "=>" in line:
                    info["libcublas"] = line.split("=>", 1)[1].strip()
                    break
    if not info["libcublas"]:
        for pattern in ("/usr/local/cuda*/targets/*/lib/libcublas.so.12",
                        "/usr/local/cuda*/lib64/libcublas.so.12"):
            hits = sorted(Path("/").glob(pattern.lstrip("/")))
            if hits:
                info["libcublas"] = str(hits[-1])
                break
    return info


def section_cuvslam(warm_up: bool) -> Report:
    info = module_version("cuvslam", dist="cuvslam")
    info.update({"dist_version": None, "get_version": None, "warm_up_gpu": None})
    if not info["importable"]:
        return info
    try:
        # __version__ is get_version()[0].split('+')[0] in v17.0.0, so it lacks the "+cu12"
        # local tag; the installed distribution version keeps it.
        from importlib import metadata
        info["dist_version"] = metadata.version("cuvslam")
    except Exception:  # noqa: BLE001
        info["dist_version"] = None
    try:
        import cuvslam  # type: ignore

        raw = cuvslam.get_version()
        # v17.0.0 pybind: returns (semantic version string, major, minor, patch).
        if isinstance(raw, (tuple, list)):
            info["get_version"] = [str(raw[0])] + [int(x) for x in raw[1:4]]
        else:
            info["get_version"] = str(raw)
    except Exception as exc:  # noqa: BLE001
        info["get_version"] = f"error: {type(exc).__name__}: {first_line(str(exc))}"
    if warm_up:
        try:
            cuvslam.warm_up_gpu()
            info["warm_up_gpu"] = "ok"
        except Exception as exc:  # noqa: BLE001
            info["warm_up_gpu"] = f"error: {type(exc).__name__}: {first_line(str(exc))}"
    return info


def section_python() -> Report:
    return {
        "version": platform.python_version(),
        "implementation": platform.python_implementation(),
        "executable": sys.executable,
        "in_venv": sys.prefix != getattr(sys, "base_prefix", sys.prefix),
        "prefix": sys.prefix,
    }


def section_packages() -> Report:
    return {
        "numpy": module_version("numpy"),
        "opencv": module_version("cv2"),
        "scipy": module_version("scipy"),
        "pyyaml": module_version("yaml"),
        "pillow": module_version("PIL"),
        "rerun-sdk": module_version("rerun", dist="rerun-sdk"),
        "evo": module_version("evo", dist="evo"),
    }


# ----------------------------------------------------------------------------- output

def summarise(report: Report) -> List[str]:
    """Human-readable list of what is missing or unexpected on this host."""
    notes: List[str] = []
    os_info, smi, cuda, cuv, dai = (report["os"], report["nvidia_smi"], report["cuda"],
                                    report["cuvslam"], report["depthai"])
    windows = os_info["system"] == "Windows"
    if not smi["present"]:
        if os_info["is_jetson"]:
            notes.append("nvidia-smi absent: normal on Jetson (integrated GPU); use jtop instead")
        elif os_info["is_wsl"]:
            notes.append("nvidia-smi missing in WSL: install/update the Windows NVIDIA driver, "
                         "then `wsl --update` and `wsl --shutdown` (never a Linux driver in WSL)")
        else:
            notes.append("nvidia-smi missing: no NVIDIA driver found")
    if not cuda["nvcc"] and not cuda["version_json_version"] and not cuda["version_txt_version"]:
        if windows:
            notes.append("CUDA toolkit not installed on Windows (not needed: cuVSLAM cannot run here)")
        elif os_info["is_jetson"]:
            notes.append("CUDA toolkit missing on Jetson: `sudo apt install nvidia-jetpack` "
                         "(methods/cuvslam-vio/tools/setup_jetson.sh does this)")
        else:
            notes.append("CUDA toolkit missing: `sudo apt install cuda-toolkit-12-6` from NVIDIA's "
                         "WSL-Ubuntu repo (methods/cuvslam-vio/tools/setup_wsl2.sh does this)")
    if os_info["is_wsl"] and cuda["libcuda"] and not cuda["libcuda"].startswith("/usr/lib/wsl/lib"):
        notes.append(f"libcuda.so.1 comes from {cuda['libcuda']}, not /usr/lib/wsl/lib: a Linux "
                     "driver was installed inside WSL; remove it (NVIDIA: Windows driver only)")
    if not cuv["importable"]:
        if windows:
            notes.append("cuvslam not importable: expected on Windows (no Windows wheel exists); "
                         "tests use cuvslam_oak.fake_cuvslam")
        else:
            notes.append(f"cuvslam not importable: {cuv['error']}")
    elif isinstance(cuv.get("warm_up_gpu"), str) and cuv["warm_up_gpu"].startswith("error"):
        notes.append(f"cuvslam.warm_up_gpu() failed: {cuv['warm_up_gpu']}")
    if not dai["importable"]:
        notes.append(f"depthai not importable: {dai['error']}")
    for name in ("numpy", "opencv"):
        if not report["packages"][name]["importable"]:
            notes.append(f"{name} not importable (needed by cuvslam_oak and by the cuVSLAM examples)")
    for name in ("scipy", "pyyaml"):
        if not report["packages"][name]["importable"]:
            notes.append(f"{name} not installed (needed only by NVIDIA's cuVSLAM examples; "
                         "cuvslam_oak does not use it)")
    if not report["packages"]["pillow"]["importable"]:
        notes.append("pillow not installed (needed only by NVIDIA's examples/euroc loader)")
    for name in ("rerun-sdk", "evo"):
        if not report["packages"][name]["importable"]:
            notes.append(f"{name} not installed (optional: visualisation / evaluation)")
    return notes


def print_human(report: Report) -> None:
    os_info = report["os"]
    print(f"os: {os_info['platform']} ({os_info['machine']})")
    if os_info["distribution"]:
        print(f"os.distribution: {os_info['distribution']}")
    print(f"os.wsl: {'yes (' + str(os_info['wsl_distro']) + ')' if os_info['is_wsl'] else 'no'}")
    if os_info["is_jetson"]:
        print(f"os.jetson: yes model={os_info['jetson_model']} L4T={os_info['l4t_release']} "
              f"jetpack={os_info['jetpack'] or 'nvidia-jetpack not installed'}")
    else:
        print("os.jetson: no")
    py = report["python"]
    print(f"python: {py['version']} ({py['implementation']}) venv={'yes' if py['in_venv'] else 'no'} "
          f"exe={py['executable']}")
    smi = report["nvidia_smi"]
    if smi["present"]:
        gpus = "; ".join(f"{g['name']} (compute {g['compute_cap']})" for g in smi["gpus"]) or "?"
        print(f"nvidia-smi: present ({smi['path']}) driver={smi['driver_version']} "
              f"driver_cuda_version={smi['driver_cuda_version']} gpu={gpus}")
        if smi["error"]:
            print(f"nvidia-smi.error: {smi['error']}")
    else:
        print("nvidia-smi: not found")
    cuda = report["cuda"]
    print(f"cuda.nvcc: {cuda['nvcc'] or 'not found'}"
          + (f" version={cuda['nvcc_version']}" if cuda["nvcc_version"] else ""))
    print(f"cuda.version_json: {cuda['version_json'] or 'not found'}"
          + (f" version={cuda['version_json_version']}" if cuda["version_json_version"] else ""))
    if cuda["version_txt_version"]:
        print(f"cuda.version_txt: {cuda['version_txt_version']}")
    print(f"cuda.dirs: {', '.join(cuda['cuda_dirs']) or 'none'}"
          + (f" (/usr/local/cuda -> {cuda['cuda_symlink']})" if cuda["cuda_symlink"] else ""))
    print(f"cuda.libcuda: {cuda['libcuda'] or 'not found'}")
    print(f"cuda.libcublas: {cuda['libcublas'] or 'not found'}")
    cuv = report["cuvslam"]
    if cuv["importable"]:
        print(f"cuvslam: importable version={cuv['version']} dist={cuv['dist_version']} "
              f"get_version={cuv['get_version']} file={cuv['file']}")
        if cuv["warm_up_gpu"] is not None:
            print(f"cuvslam.warm_up_gpu: {cuv['warm_up_gpu']}")
    else:
        print(f"cuvslam: NOT importable ({cuv['error']})")
    dai = report["depthai"]
    if dai["importable"]:
        print(f"depthai: importable version={dai['version']}")
    else:
        print(f"depthai: NOT importable ({dai['error']})")
    for name, info in report["packages"].items():
        if info["importable"]:
            print(f"{name}: {info['version']}")
        else:
            print(f"{name}: not installed")
    notes = report["summary"]
    print("summary: " + ("all expected pieces present" if not notes else f"{len(notes)} note(s)"))
    for note in notes:
        print(f"  - {note}")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", action="store_true", help="print one JSON object instead of text")
    parser.add_argument("--warm-up", action="store_true",
                        help="call cuvslam.warm_up_gpu() to prove CUDA/cuBLAS/cuSOLVER initialise")
    args = parser.parse_args(argv)

    os_info = section_os()
    report: Report = {
        "os": os_info,
        "python": section_python(),
        "nvidia_smi": section_nvidia_smi(os_info["is_wsl"]),
        "cuda": section_cuda(),
        "cuvslam": section_cuvslam(args.warm_up),
        "depthai": module_version("depthai"),
        "packages": section_packages(),
    }
    report["summary"] = summarise(report)
    if args.json:
        print(json.dumps(report, indent=2, default=str))
    else:
        print_human(report)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 - the report must never fail the caller
        print(f"check_env: unexpected error: {type(exc).__name__}: {exc}", file=sys.stderr)
        sys.exit(0)
