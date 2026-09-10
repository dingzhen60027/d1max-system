// Local-only, read-only URDF assets; separate from the live sensor bridge.
import {createServer} from 'node:http';
import {createReadStream} from 'node:fs';
import {readFile,stat} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {join} from 'node:path';
const root=fileURLToPath(new URL('../../urdf_ws/src/max_description/',import.meta.url));
const host='http://127.0.0.1:8770';
const urdf=(await readFile(join(root,'urdf/max.urdf'),'utf8')).replaceAll('package://max_description/',host+'/max_description/');
const allowedMeshes=new Set([...urdf.matchAll(/\/meshes\/([A-Za-z0-9_]+[.]STL)/g)].map(m=>m[1]));
const server=createServer(async(req,res)=>{
 res.setHeader('Access-Control-Allow-Origin','*');
 res.setHeader('X-Content-Type-Options','nosniff');
 if(req.method==='OPTIONS'){res.writeHead(204,{'Access-Control-Allow-Methods':'GET, HEAD, OPTIONS'}).end();return;}
 if(!['GET','HEAD'].includes(req.method)){res.writeHead(405).end();return;}
 const path=new URL(req.url,host).pathname;
 if(path==='/max_description/urdf/max.urdf'){
  res.writeHead(200,{'Content-Type':'application/xml','Cache-Control':'no-cache','Content-Length':Buffer.byteLength(urdf)});
  res.end(req.method==='HEAD'?undefined:urdf);return;
 }
 const match=/^\/max_description\/meshes\/([A-Za-z0-9_]+[.]STL)$/.exec(path);
 if(!match||!allowedMeshes.has(match[1])){res.writeHead(404).end();return;}
 try{
  const file=join(root,'meshes',match[1]),info=await stat(file);
  res.writeHead(200,{'Content-Type':'model/stl','Content-Length':info.size,'Cache-Control':'public, max-age=86400'});
  if(req.method==='HEAD'){res.end();return;}
  const stream=createReadStream(file);stream.on('error',()=>res.destroy());res.on('close',()=>stream.destroy());stream.pipe(res);
 }catch{res.writeHead(404).end();}
});
server.listen(8770,'127.0.0.1',()=>console.log('D1 Max original URDF/STL: '+host+'/max_description/urdf/max.urdf'));
