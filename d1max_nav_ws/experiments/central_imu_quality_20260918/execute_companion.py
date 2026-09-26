#!/usr/bin/env python3
"""Execute this small, trusted numerical companion without a Jupyter kernel."""
import contextlib
import html
import io
import json
from pathlib import Path

root = Path(__file__).resolve().parent
path = root / 'quality_checks.ipynb'
book = json.loads(path.read_text())
assert book['nbformat'] == 4
namespace = {'__name__': '__main__'}
execution = 0
sections = []
for cell in book['cells']:
    source = ''.join(cell['source'])
    assert cell['cell_type'] in ('markdown', 'code') and cell['id']
    if cell['cell_type'] == 'code':
        execution += 1
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            exec(compile(source, f'quality_checks.ipynb:{cell["id"]}', 'exec'), namespace)
        cell['execution_count'] = execution
        cell['outputs'] = [{'output_type': 'stream', 'name': 'stdout', 'text': output.getvalue().splitlines(keepends=True)}]
        sections.append('<details><summary>Python calculation</summary><pre>' + html.escape(source) + '</pre></details><pre>' + html.escape(output.getvalue()) + '</pre>')
        print(output.getvalue(), end='')
    else:
        sections.append('<pre class="prose">' + html.escape(source) + '</pre>')
book['metadata']['execution_audit'] = {
    'method': 'Trusted cells compiled and executed top-to-bottom in a fresh shared Python namespace; no Jupyter kernel.',
    'code_cells_executed': execution,
    'all_cells_succeeded': True,
    'presentation': 'HTML preview generated; external notebook UI rendering not certified.'}
path.write_text(json.dumps(book, ensure_ascii=False, indent=1) + '\n')
preview = '<!doctype html><html lang="zh"><meta charset="utf-8"><title>中心 IMU 质量复核</title><style>body{max-width:1000px;margin:32px auto;padding:0 24px;font:16px system-ui;line-height:1.5;color:#222;background:#fff}pre{overflow:auto;padding:12px;background:#f4f5f6;font-size:14px}.prose{font:inherit;white-space:pre-wrap;background:transparent}summary{cursor:pointer;margin-top:18px}</style><body>' + ''.join(sections) + '</body></html>'
(root / 'quality_checks_preview.html').write_text(preview)
print('Executed', execution, 'cells and saved outputs.')
