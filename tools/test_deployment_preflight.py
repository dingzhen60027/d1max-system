import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('deployment_preflight',
    Path(__file__).with_name('deployment_preflight.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PreflightTests(unittest.TestCase):
    def test_clone_does_not_claim_runtime_or_physical_acceptance(self):
        with tempfile.TemporaryDirectory() as d:
            repo = Path(d)
            report = module.inventory(repo, 'source', environment={})
            self.assertFalse(report['passed'])
            self.assertFalse(report['runtime']['sealed_runtime_verified'])
            self.assertFalse(report['runtime']['physical_acceptance_verified'])
            self.assertFalse(report['external_data']['included_in_git'])

    def test_explicit_roots_and_release_take_precedence(self):
        with tempfile.TemporaryDirectory() as d:
            repo = Path(d)
            nav, app, release = (repo / p for p in ('nav', 'app', 'release'))
            release.mkdir()
            (release / 'release.json').write_text('{}')
            report = module.inventory(repo, 'source', environment={
                'D1MAX_NAV_ROOT': str(nav), 'D1MAX_APP_ROOT': str(app), 'D1MAX_RELEASE': str(release)})
            self.assertEqual(report['roots']['nav'], str(nav))
            self.assertEqual(report['roots']['app'], str(app))
            self.assertTrue(report['runtime']['descriptor_present'])
            self.assertFalse(report['runtime']['sealed_runtime_verified'])

    def test_selector_traversal_is_not_followed(self):
        with tempfile.TemporaryDirectory() as d:
            repo = Path(d)
            deploy = repo / 'd1max_nav_ws/deploy'
            deploy.mkdir(parents=True)
            (deploy / 'single_floor_release.json').write_text(json.dumps({'release_directory': '../other'}))
            report = module.inventory(repo, 'source', environment={})
            self.assertIsNone(report['runtime']['selected_release'])
            self.assertEqual(report['runtime']['selector_error'], 'invalid selector')

    def test_repository_sources_are_complete(self):
        report = module.inventory(Path(__file__).resolve().parents[1], 'source', environment={})
        self.assertTrue(report['passed'], report['checks'])


if __name__ == '__main__':
    unittest.main()
