import subprocess

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class InstallXarmBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):
        pass

    def finalize(self, version, build_data, artifact_paths):
        # Note that this requires "pip" to be available during hatchling build.
        # Check the pyproject build-system requires.
        command = ["pip", "install", "third_party/xarm_py"]
        subprocess.run(command, check=True)
