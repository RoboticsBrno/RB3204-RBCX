import sys
import subprocess
import re

Import("env")

TAG_RE = rb"fw_v([0-9]+)\.([0-9]+)\.([0-9]+)"

def add_version_defines():
    defines = {}

    try:
        revision = subprocess.check_output([ "git", "rev-parse", "HEAD" ])
        defines["RBCX_VER_REVISION"] = '\\"%s\\"' % revision[:8].decode("utf-8")

        diff_status = subprocess.call([ "git", "diff", "--quiet" ])
        defines["RBCX_VER_DIRTY"] = 0 if diff_status == 0 else 1
        defines["RBCX_VER_DIRTY_STR"] = '\\"\\"' if diff_status == 0 else '\\"-dirty\\"'
    except subprocess.CalledProcessError:
        print("Calling git failed, no version info available.")
        return

    try:
        tag = subprocess.check_output([ "git", "describe", "--tags", "--match", "fw_*", "--abbrev=0"])
        m = re.match(TAG_RE, tag)
        if not m:
            print("WARNING: Invalid fw tag, got %s expected %s" % (tag, TAG_RE))
        else:
            defines["RBCX_VER_NUMBER"] = (int(m.group(1)) << 16) | (int(m.group(2)) << 8) | int(m.group(3))
    except subprocess.CalledProcessError:
        defines["RBCX_VER_NUMBER"] = 0

    print("RBCX VERSION: 0x%08x %s" % (defines["RBCX_VER_NUMBER"], defines))

    defines = { ("-D%s=" % k): defines[k] for k in defines }

    newflags = []
    for f in env.get("BUILD_FLAGS", []):
        found = False
        for k in defines:
            if f.startswith(k):
                newflags.append("%s%s" % (k, defines[k]))
                found = True
                break
        if not found:
            newflags.append(f)

    env.Replace(BUILD_FLAGS=newflags)

add_version_defines()
