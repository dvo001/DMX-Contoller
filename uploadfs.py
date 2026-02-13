Import("env")
from subprocess import run
from SCons.Script import Exit

def after_upload(source, target, env):
    print("Uploading SPIFFS for env:", env["PIOENV"])
    cmd = [
        env.subst("$PYTHONEXE"),
        "-m",
        "platformio",
        "run",
        "-e",
        env["PIOENV"],
        "-t",
        "uploadfs",
    ]
    result = run(cmd, check=False)
    if result.returncode != 0:
        print("SPIFFS upload failed")
        Exit(result.returncode)

env.AddPostAction("upload", after_upload)
