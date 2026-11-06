import json
from pathlib import Path
import subprocess
import tempfile
import unittest


class BoundedDiagnosticFileTest(unittest.TestCase):
    def test_idle_cannot_overwrite_task_tail_and_task_history_is_bounded(self):
        package = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix='d1max-task-trace-test-') as tmp:
            root = Path(tmp)
            exe = root/'probe'
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(package/'include'), str(package/'test/task_diagnostic_file_probe.cpp'),
                            '-o', str(exe)], check=True)
            result = subprocess.run([str(exe), str(root/'trace')], check=True, text=True, capture_output=True)
            directory = Path(result.stdout.strip())
            index = json.loads((directory/'task-index.json').read_text())
            self.assertEqual(index['evicted_tasks'], 25)
            keys = [bytes.fromhex(row['key_hex']).decode() for row in index['tasks']]
            self.assertEqual(set(keys), {f'7:sessiontask{i}' for i in range(26, 30)})
            self.assertEqual(directory.stat().st_mode & 0o777, 0o700)
            self.assertTrue(list(directory.glob('idle-*')))

    def test_rotation_no_throw_and_exact_source_time(self):
        package = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix='d1max-trace-test-') as tmp:
            root = Path(tmp)
            exe = root / 'probe'
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(package/'include'), str(package/'test/bounded_diagnostic_file_probe.cpp'),
                            '-o', str(exe)], check=True)
            result = subprocess.run([str(exe), str(root/'trace')], check=True, text=True, capture_output=True)
            first, second = map(Path, result.stdout.splitlines())
            files = list(first.glob('trace-*.jsonl'))
            self.assertEqual(len(files), 4)
            self.assertLessEqual(sum(f.stat().st_size for f in files), 4*512)
            self.assertEqual(first.stat().st_mode & 0o777, 0o700)
            records = [json.loads(line) for f in files for line in f.read_text().splitlines()]
            ordered = sorted(records, key=lambda r: r['batch_sequence'])
            self.assertEqual(ordered[-1]['payload']['seq'], 249)
            self.assertTrue(all(r['payload']['source_ns'] == 1234567890123456789 for r in ordered))
            self.assertGreater(ordered[-1]['overwritten_slots'], 0)
            self.assertTrue((first/'index.json').exists() and (second/'index.json').exists())
            self.assertEqual((root/'trace/blocker').read_text(), 'unchanged')


if __name__ == '__main__':
    unittest.main()
