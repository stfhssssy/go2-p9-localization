from pathlib import Path
from setuptools import setup, find_packages

root = Path(__file__).parent
setup(name='go2_p9', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/go2_p9']),
                  ('share/go2_p9', ['package.xml']),
                  ('share/go2_p9/launch', ['launch/go2_p9.launch.py'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='stfhssssy', maintainer_email='stfhssssy@users.noreply.github.com',
      description='Go2 P9 standalone sensor interfaces and geometry mapping', license='GPL-2.0-only',
      entry_points={'console_scripts': [
          'p9 = go2_p9.cli:main', 'p9_adapter = go2_p9.adapter_node:main',
          'p9_mapping = go2_p9.mapping_node:main', 'p9_diagnose = go2_p9.diagnose:main']})
