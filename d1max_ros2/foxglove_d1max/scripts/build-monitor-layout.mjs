import {readFile,writeFile} from 'node:fs/promises';
import {execFileSync} from 'node:child_process';
import {monitorLayout} from './monitor-layout.mjs';
const metadata=JSON.parse(execFileSync('python3',['scripts/pcd_map_publisher.py','--inspect'],{encoding:'utf8'}));
const panel=JSON.parse(await readFile('config/panel.json','utf8'));
const source=JSON.parse(await readFile('layouts/D1Max-Live.json','utf8'));
await writeFile('layouts/D1Max-Monitor.json',JSON.stringify(monitorLayout(source,metadata,panel),null,2)+'\n');
await writeFile('artifacts/monitor-map-metadata.json',JSON.stringify(metadata,null,2)+'\n');
console.log('Generated monitor-only layout; PCD and live data have separate frames.');
