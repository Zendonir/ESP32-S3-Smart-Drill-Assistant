#!/usr/bin/env python3
"""Assembles the browser flasher site (ESP Web Tools) from a finished PlatformIO build.

Usage:
    pio run -e sda_s3_touch_128
    python3 scripts/build_flasher.py [--out _site] [--env sda_s3_touch_128]

The output directory contains index.html, manifest.json, build-info.json and the
binaries (bootloader, partition table, boot_app0, application). It can be served by
any HTTPS web server, e.g. GitHub Pages (see .github/workflows/pages.yml).

Flash layout (ESP32-S3, matches partitions.csv):
    0x0000   bootloader.bin
    0x8000   partitions.bin
    0xE000   boot_app0.bin   (otadata: boot from app0)
    0x10000  firmware.bin    (app0)
"""
import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request

ESP_WEB_TOOLS_VERSION = "10.2.1"

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def find_boot_app0():
    candidates = [
        os.path.join(os.path.expanduser("~"), ".platformio", "packages", "framework-arduinoespressif32",
                     "tools", "partitions", "boot_app0.bin"),
    ]
    core = os.environ.get("PLATFORMIO_CORE_DIR")
    if core:
        candidates.insert(0, os.path.join(core, "packages", "framework-arduinoespressif32", "tools",
                                          "partitions", "boot_app0.bin"))
    for c in candidates:
        if os.path.isfile(c):
            return c
    sys.exit("boot_app0.bin not found (is the Arduino-ESP32 framework installed by PlatformIO?)")


def firmware_version():
    with open(os.path.join(ROOT, "include", "Version.h"), encoding="utf-8") as f:
        m = re.search(r'#define\s+SDA_FW_VERSION_STR\s+"([^"]+)"', f.read())
    return m.group(1) if m else "0.0.0"


def git_commit():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def install_esp_web_tools(out_dir, tarball=None):
    """Copies the ESP Web Tools browser bundle (dist/web) next to index.html so the
    flasher does not depend on a CDN. Source: npm package esp-web-tools (pinned)."""
    dest = os.path.join(out_dir, "esp-web-tools")
    if tarball is None:
        url = f"https://registry.npmjs.org/esp-web-tools/-/esp-web-tools-{ESP_WEB_TOOLS_VERSION}.tgz"
        tarball = os.path.join(tempfile.mkdtemp(), "esp-web-tools.tgz")
        urllib.request.urlretrieve(url, tarball)
    shutil.rmtree(dest, ignore_errors=True)
    os.makedirs(dest)
    prefix = "package/dist/web/"
    with tarfile.open(tarball, "r:gz") as tf:
        for m in tf.getmembers():
            name = m.name
            if not m.isfile() or not name.startswith(prefix):
                continue
            rel = name[len(prefix):]
            if not rel or "/" in rel or rel.startswith("."):
                continue  # flat bundle only, no path traversal
            with tf.extractfile(m) as src, open(os.path.join(dest, rel), "wb") as dst:
                shutil.copyfileobj(src, dst)
    if not os.path.isfile(os.path.join(dest, "install-button.js")):
        sys.exit("esp-web-tools bundle incomplete")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "_site"))
    ap.add_argument("--env", default="sda_s3_touch_128")
    ap.add_argument("--esp-web-tools-tgz", default=None,
                    help=f"local esp-web-tools-{ESP_WEB_TOOLS_VERSION}.tgz (default: download from npm)")
    args = ap.parse_args()

    build = os.path.join(ROOT, ".pio", "build", args.env)
    parts = [
        ("bootloader.bin", os.path.join(build, "bootloader.bin"), 0x0),
        ("partitions.bin", os.path.join(build, "partitions.bin"), 0x8000),
        ("boot_app0.bin", find_boot_app0(), 0xE000),
        ("firmware.bin", os.path.join(build, "firmware.bin"), 0x10000),
    ]
    for _, src, _ in parts:
        if not os.path.isfile(src):
            sys.exit(f"missing {src} - run 'pio run -e {args.env}' first")
    # ESP32 images start with the magic byte 0xE9 (bootloader and application)
    for name, src, _ in (parts[0], parts[3]):
        with open(src, "rb") as f:
            if f.read(1) != b"\xE9":
                sys.exit(f"{name} is not a valid ESP32 image")

    os.makedirs(args.out, exist_ok=True)
    shutil.copy(os.path.join(ROOT, "flasher", "index.html"), os.path.join(args.out, "index.html"))
    install_esp_web_tools(args.out, args.esp_web_tools_tgz)
    for name, src, _ in parts:
        shutil.copy(src, os.path.join(args.out, name))

    version = firmware_version()
    commit = git_commit()
    manifest = {
        "name": "Smart Drill Assistant",
        "version": f"{version} ({commit})",
        "home_assistant_domain": None,
        "new_install_prompt_erase": True,
        "builds": [{
            "chipFamily": "ESP32-S3",
            "parts": [{"path": name, "offset": off} for name, _, off in parts],
        }],
    }
    del manifest["home_assistant_domain"]
    with open(os.path.join(args.out, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    info = {
        "version": version,
        "commit": commit,
        "date": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
        "env": args.env,
        "parts": [{"path": name, "offset": f"0x{off:X}", "size": os.path.getsize(src), "sha256": sha256(src)}
                  for name, src, off in parts],
    }
    with open(os.path.join(args.out, "build-info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)
    # GitHub Pages: serve files as they are (no Jekyll processing)
    open(os.path.join(args.out, ".nojekyll"), "w").close()
    print(f"flasher site written to {args.out} (version {version}, commit {commit})")


if __name__ == "__main__":
    main()
