import { build } from 'esbuild';
import { mkdir, copyFile, readFile } from 'node:fs/promises';
let token="";
try{token=JSON.parse(await readFile("config/manager.local.json","utf8")).token;}catch{}
await mkdir('dist', {recursive:true});
await mkdir('artifacts/preview', {recursive:true});
await build({entryPoints:['src/index.tsx'], bundle:true, outfile:'dist/extension.js', platform:'browser', format:'cjs', target:'es2022', minify:true, loader:{'.css':'text'}, define:{'process.env.NODE_ENV':'"production"','__D1MAX_MANAGER_TOKEN__':JSON.stringify(token)}});
await build({entryPoints:['src/preview.tsx'], bundle:true, outfile:'artifacts/preview/preview.js', platform:'browser', format:'iife', target:'es2022', loader:{'.css':'text'}, define:{'process.env.NODE_ENV':'"production"'}});
await copyFile('preview.html','artifacts/preview/preview.html');
console.log('Built extension and isolated preview (preview never connects to ROS).');
