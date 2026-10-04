from setuptools import setup
from glob import glob
setup(name='d1max_pct_scan',version='0.1.0',
 packages=['d1max_pct_scan','d1max_pct_scan.pointcloud_helpers'],
 # Install the original audited helpers verbatim, from their single source.
 # Runtime imports must not depend on a workspace CWD or inject tools/ into
 # sys.path. The legacy offline tools remain usable from their original home.
 package_dir={'d1max_pct_scan.pointcloud_helpers':'../../tools/pointcloud_preprocessing'},
 data_files=[('share/ament_index/resource_index/packages',['resource/d1max_pct_scan']),
 ('share/d1max_pct_scan',['package.xml','PERCEPTION_RAY_PROJECTOR.md']),('share/d1max_pct_scan/config',glob('config/*')),
 ('share/d1max_pct_scan/launch',glob('launch/*.launch.py'))],
 entry_points={'console_scripts':['pct_scan_simulator=d1max_pct_scan.simulation:main',
 'pct_scan_run=d1max_pct_scan.runtime:main','pct_scan_verify=d1max_pct_scan.verify:main',
 'pct_scan_trajectory_guard=d1max_pct_scan.trajectory_guard:main',
 'navigation_session=d1max_pct_scan.navigation_session:main',
 # Historical CLI alias; both names have the same implementation/task owner.
 'single_floor_session=d1max_pct_scan.single_floor_session:main',
 'continuous_reference=d1max_pct_scan.continuous_reference_node:main',
 'execution_safety=d1max_pct_scan.execution_safety_node:main']})
