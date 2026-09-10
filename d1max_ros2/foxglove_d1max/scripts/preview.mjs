import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
const root=fileURLToPath(new URL('../artifacts/preview/',import.meta.url));
const allowed=new Map([['/','preview.html'],['/preview.html','preview.html'],['/preview.js','preview.js']]);
createServer(async(req,res)=>{
  const file=allowed.get(new URL(req.url,'http://localhost').pathname);
  if(!file){res.writeHead(404).end();return;}
  try{const data=await readFile(root+file);res.writeHead(200,{'Content-Type':file.endsWith('.js')?'text/javascript; charset=utf-8':'text/html; charset=utf-8','Cache-Control':'no-store'}).end(data);}catch{res.writeHead(500).end('Run npm run build first.');}
}).listen(8768,'127.0.0.1',()=>process.stdout.write('Isolated UI preview: http://127.0.0.1:8768 (no ROS connection)\n'));
