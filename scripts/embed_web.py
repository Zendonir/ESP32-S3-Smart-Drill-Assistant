# Pre-build script (PlatformIO/SCons):
#  1. embeds the web UI into the firmware: data/web/index.html is gzip compressed
#     and written as a C array to include/WebAssets.h. Embedding (instead of a
#     LittleFS upload) guarantees that firmware and web UI always match, also
#     after OTA updates.
#  2. passes the git commit to the firmware (SDA_FW_COMMIT, shown under
#     System > Firmware, in the diagnostics, the log and /api/status).
import gzip
import os
import subprocess

try:
    Import("env")  # noqa: F821  (provided by SCons)
    PROJECT_DIR = env["PROJECT_DIR"]  # noqa: F821
    HAVE_ENV = True
except NameError:  # executed stand-alone
    PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    HAVE_ENV = False

SRC = os.path.join(PROJECT_DIR, "data", "web", "index.html")
DST = os.path.join(PROJECT_DIR, "include", "WebAssets.h")


def generate():
    with open(SRC, "rb") as f:
        raw = f.read()
    gz = gzip.compress(raw, compresslevel=9, mtime=0)
    lines = []
    for i in range(0, len(gz), 20):
        lines.append("    " + ", ".join("0x%02X" % b for b in gz[i:i + 20]) + ",")
    content = (
        "// GENERATED FILE - do not edit. Source: data/web/index.html (scripts/embed_web.py)\n"
        "#pragma once\n#include <cstddef>\n#include <cstdint>\n\nnamespace web_assets {\n"
        "// gzip compressed index.html (%d bytes raw)\n"
        "alignas(4) static const uint8_t kIndexHtmlGz[] = {\n%s\n};\n"
        "static constexpr size_t kIndexHtmlGzLen = sizeof(kIndexHtmlGz);\n}  // namespace web_assets\n"
    ) % (len(raw), "\n".join(lines))
    old = None
    if os.path.exists(DST):
        with open(DST, "r") as f:
            old = f.read()
    if old != content:
        with open(DST, "w") as f:
            f.write(content)
        print("embed_web: %s -> %s (%d -> %d bytes)" % (SRC, DST, len(raw), len(gz)))


def inject_commit():
    try:
        commit = subprocess.check_output(["git", "-C", PROJECT_DIR, "rev-parse", "--short", "HEAD"],
                                         text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return  # no git: firmware reports "unknown"
    if HAVE_ENV:
        env.Append(CPPDEFINES=[("SDA_FW_COMMIT", '\\"%s\\"' % commit)])  # noqa: F821


generate()
inject_commit()
