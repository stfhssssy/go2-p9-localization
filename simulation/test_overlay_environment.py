#!/usr/bin/env python3
"""Exercise inside.sh with duplicate-preserving ament-style environment hooks.

No ROS installation is needed. Only the three absolute environment roots are
redirected to temporary fixtures; the wrapper logic itself executes unchanged.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
VARIABLES = ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
             'PYTHONPATH', 'PATH', 'LD_LIBRARY_PATH', 'PKG_CONFIG_PATH',
             'ROS_PACKAGE_PATH', 'GZ_SIM_RESOURCE_PATH', 'GZ_SIM_SYSTEM_PLUGIN_PATH')


def hook(prefix, trace):
    # This deliberately does NOT move an existing value, matching the observed
    # colcon/ament bug trigger when a preloaded Docker overlay is sourced twice.
    body = ['export P9_TEST_TRACE="${P9_TEST_TRACE-}'+trace+';"']
    for variable in VARIABLES:
        value = str(prefix/variable.lower())
        body += [f'case ":${{{variable}-}}:" in',
                 f'  *":{value}:"*) ;;',
                 f'  *) export {variable}="{value}${{{variable}:+:${variable}}}" ;;',
                 'esac']
    return '\n'.join(body)+'\n'


class OverlayEnvironmentTests(unittest.TestCase):
    def fixture(self, preloaded, duplicate=False):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ros, old, new = root/'ros', root/'old', root/'new'
            for path in (ros, old, new):
                path.mkdir()
            (ros/'setup.bash').write_text(hook(ros, 'ros'))
            (old/'setup.bash').write_text(hook(old, 'underlay'))
            (new/'local_setup.bash').write_text(hook(new, 'new-local'))
            # Failure if wrapper accidentally reverts to replaying setup chain.
            (new/'setup.bash').write_text('echo unexpected-new-setup >&2\nexit 87\n')
            wrapper = (HERE/'inside.sh').read_text()
            for actual, fixture in [('/opt/ros/jazzy', ros), ('/ws/install', old),
                                    ('/opt/p9_ws/install', new)]:
                wrapper = wrapper.replace(actual, str(fixture))
            script = root/'inside.sh'
            script.write_text(wrapper)
            environment = dict(os.environ)
            for variable in VARIABLES:
                # Keep unrelated entries and similarly named sibling prefixes.
                original = f'{root}/new-neighbor/value:/usr/bin:/bin'
                if preloaded:
                    original = str(new/variable.lower())+':'+original
                if duplicate:
                    original += ':'+str(new/variable.lower())
                environment[variable] = original
            environment.pop('P9_TEST_TRACE', None)
            code = 'import os,json; print(json.dumps(dict(os.environ)))'
            result = subprocess.run(['/bin/bash', str(script), sys.executable, '-c', code],
                                    text=True, capture_output=True, env=environment, check=True)
            values = json.loads(result.stdout)
            self.assertEqual(values['P9_TEST_TRACE'], 'ros;underlay;new-local;')
            for variable in VARIABLES:
                with self.subTest(variable=variable):
                    entries = values[variable].split(':')
                    self.assertEqual(entries[0], str(new/variable.lower()))
                    self.assertEqual(entries.count(str(new/variable.lower())), 1)
                    self.assertLess(entries.index(str(new/variable.lower())), entries.index(str(old/variable.lower())))
                    self.assertIn(str(root/'new-neighbor/value'), entries)
                    self.assertIn('/usr/bin', entries)
            self.assertEqual(values['ROS_DOMAIN_ID'], '94')
            self.assertEqual(values['GZ_PARTITION'], 'ssy-p9-nav-20261110')

    def test_fresh_shell(self):
        self.fixture(False)

    def test_entrypoint_overlay_already_sourced(self):
        self.fixture(True)

    def test_remove_all_repeated_overlay_entries(self):
        self.fixture(True, duplicate=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
