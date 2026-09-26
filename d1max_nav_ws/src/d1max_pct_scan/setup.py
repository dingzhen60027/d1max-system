from setuptools import setup
from glob import glob
setup(name='d1max_pct_scan',version='0.1.0',packages=['d1max_pct_scan'],
 data_files=[('share/ament_index/resource_index/packages',['resource/d1max_pct_scan']),
 ('share/d1max_pct_scan',['package.xml','PERCEPTION_RAY_PROJECTOR.md']),('share/d1max_pct_scan/config',glob('config/*')),
 ('share/d1max_pct_scan/launch',glob('launch/*.launch.py'))],
 entry_points={'console_scripts':['pct_scan_simulator=d1max_pct_scan.simulation:main',
 'pct_scan_run=d1max_pct_scan.runtime:main','pct_scan_verify=d1max_pct_scan.verify:main',
 'pct_scan_trajectory_guard=d1max_pct_scan.trajectory_guard:main']})
