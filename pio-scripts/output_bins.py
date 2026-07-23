Import('env')
import os
import shutil
import gzip
import json

OUTPUT_DIR = "build_output{}".format(os.path.sep)
#OUTPUT_DIR = os.path.join("build_output")

def _get_cpp_define_value(env, define):
    define_list = [item[-1] for item in env["CPPDEFINES"] if item[0] == define]

    if define_list:
        return define_list[0]

    return None

def _create_dirs(dirs=["map", "release", "firmware"]):
    for d in dirs:
        os.makedirs(os.path.join(OUTPUT_DIR, d), exist_ok=True)

def create_release(source):
    # Operational tools use the stable environment filename, while releases use
    # the embedded hardware-family name. Both must point at the same fresh image.
    variant = env["PIOENV"]
    bin_file = "{}firmware{}{}.bin".format(OUTPUT_DIR, os.path.sep, variant)
    print(f"Copying {source} to {bin_file}")
    shutil.copy(source, bin_file)

    release_name_def = _get_cpp_define_value(env, "WLED_RELEASE_NAME")
    if release_name_def:
        release_name = release_name_def.replace("\\\"", "")
        with open("package.json", "r") as package:
            version = json.load(package)["version"]        
        release_file = os.path.join(OUTPUT_DIR, "release", f"WLED_{version}_{release_name}.bin")
        release_gz_file = release_file + ".gz"
        print(f"Copying {source} to {release_file}")
        shutil.copy(source, release_file)
        bin_gzip(release_file, release_gz_file)

def copy_build_outputs(source, target, env):
    _create_dirs()
    variant = env["PIOENV"]
    builddir = os.path.join(env["PROJECT_BUILD_DIR"],  variant)
    firmware_file = os.path.join(builddir, env["PROGNAME"] + ".bin")
    source_map = os.path.join(builddir, env["PROGNAME"] + ".map")

    # create string with location and file names based on variant
    map_file = "{}map{}{}.map".format(OUTPUT_DIR, os.path.sep, variant)

    create_release(firmware_file)

    # copy firmware.map to map/<variant>.map
    if os.path.isfile("firmware.map"):
        print("Found linker mapfile firmware.map")
        shutil.copy("firmware.map", map_file)
    if os.path.isfile(source_map):
        print(f"Found linker mapfile {source_map}")
        shutil.copy(source_map, map_file)

def bin_gzip(source, target):
    # only create gzip for esp8266
    if not env["PIOPLATFORM"] == "espressif8266":
        return
    
    print(f"Creating gzip file {target} from {source}")
    with open(source,"rb") as fp:
        with gzip.open(target, "wb", compresslevel = 9) as f:
            shutil.copyfileobj(fp, f)

# PlatformIO may restore firmware.bin directly from its build cache, which skips
# file-level post actions. Use an always-run alias so every normal build refreshes
# the operational and release artifacts after firmware.bin is available.
package_outputs = env.Alias(
    "package_outputs",
    "$BUILD_DIR/${PROGNAME}.bin",
    copy_build_outputs,
)
env.AlwaysBuild(package_outputs)
env.Default(package_outputs)
