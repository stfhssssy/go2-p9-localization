from glob import glob
from setuptools import find_packages, setup
setup(name='go2_p9_navigation',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/go2_p9_navigation']),('share/go2_p9_navigation',['package.xml'])]+[(f'share/go2_p9_navigation/{d}',glob(f'{d}/*')) for d in ('launch','config','provenance')],
 install_requires=['setuptools'],zip_safe=True,maintainer='stfhssssy',maintainer_email='stfhssssy@users.noreply.github.com',
 description='P9 localization/mapping with Zhang Ji terrain, FAR and local planner interfaces',license='GPL-2.0-only',
 entry_points={'console_scripts':['p9nav = go2_p9_navigation.cli:main','p9_navigation_guard = go2_p9_navigation.guard_node:main']})
