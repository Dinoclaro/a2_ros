"""Subprocess management for explore/nav ROS 2 launch stacks."""

from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory


def topic_publisher_count(topic: str) -> int:
    """Return publisher count for a topic via ``ros2 topic info``."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'info', topic],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (subprocess.SubprocessError, OSError):
        return 0

    match = re.search(r'Publisher count:\s*(\d+)', result.stdout)
    return int(match.group(1)) if match else 0


def node_running(name_substring: str) -> bool:
    """Return True if any node name contains ``name_substring``."""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (subprocess.SubprocessError, OSError):
        return False

    return any(name_substring in line for line in result.stdout.splitlines())


class StackManager:
    """Spawn and kill a single explore or nav launch subprocess."""

    def __init__(self) -> None:
        self._proc: Optional[subprocess.Popen] = None
        self._log_handle: Optional[object] = None
        self._log_path: Optional[str] = None

    @property
    def log_path(self) -> Optional[str]:
        """Path to the active stack log file, if any."""
        return self._log_path

    def is_running(self) -> bool:
        """True while the subprocess is alive."""
        return self._proc is not None and self._proc.poll() is None

    def has_exited(self) -> bool:
        """True if a subprocess was started and has since exited."""
        return self._proc is not None and self._proc.poll() is not None

    def spawn_stack(
        self,
        launch_package: str,
        launch_name: str,
        launch_args: List[str],
        save_dir: str,
    ) -> None:
        """Launch ``ros2 launch <package> <launch_name>`` and log output."""
        if self.is_running():
            raise RuntimeError('A stack is already running')

        share_dir = get_package_share_directory(launch_package)
        launch_file = os.path.join(share_dir, 'launch', launch_name)
        if not os.path.isfile(launch_file):
            # Allow params like "launch/exploration.launch.py".
            alt_path = os.path.join(share_dir, launch_name)
            if os.path.isfile(alt_path):
                launch_file = alt_path
            else:
                raise FileNotFoundError(f'Launch file not found: {launch_file}')

        stack_tag = 'explore' if 'exploration' in launch_name else 'nav'
        os.makedirs(save_dir, exist_ok=True)
        self._log_path = os.path.join(save_dir, f'{stack_tag}_launch.log')
        self._log_handle = open(
            self._log_path, 'a', encoding='utf-8', buffering=1
        )
        self._log_handle.write(
            f'\n--- spawn {time.strftime("%Y-%m-%d %H:%M:%S")} ---\n'
        )

        cmd = ['ros2', 'launch', launch_package, launch_name, *launch_args]
        self._log_handle.write(f'cmd: {" ".join(cmd)}\n')

        self._proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
        )

    def kill_stack(self) -> None:
        """Send SIGINT to the process group, then SIGKILL after 10 s."""
        if self._proc is None:
            return

        if self._proc.poll() is not None:
            self._cleanup()
            return

        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        except ProcessLookupError:
            self._cleanup()
            return

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                break
            time.sleep(0.2)
        else:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass

        self._cleanup()

    def _cleanup(self) -> None:
        self._proc = None
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None
