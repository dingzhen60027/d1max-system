// Execute the button's actual handler with local deferred I/O; never call a backend.
import assert from 'node:assert/strict'
import {readFileSync} from 'node:fs'
import test from 'node:test'
import {parse} from '@babel/parser'

const source=readFileSync(new URL('../src/RemoteLaunchButton.jsx',import.meta.url),'utf8')
const component=parse(source,{sourceType:'module',plugins:['jsx']}).program.body
 .find(node=>node.type==='ExportDefaultDeclaration').declaration
const handler=component.body.body.flatMap(node=>node.declarations||[])
 .find(node=>node.id.name==='openRemote').init

function fixture(){
 const requests=[],timeouts=[],pending={current:false},state={opening:false,error:''}
 const openRemote=Function('pending','setOpening','setError','fetch','AbortSignal',
  'return ('+source.slice(handler.start,handler.end)+')')(
  pending,value=>{state.opening=value},value=>{state.error=value},
  (path,options)=>new Promise((resolve,reject)=>requests.push({path,options,resolve,reject})),
  {timeout:ms=>{timeouts.push(ms);return 'mock-signal'}},
 )
 return {openRemote,requests,timeouts,pending,state}
}

test('opening sends only POST {} and blocks repeated clicks until completion',async()=>{
 const f=fixture(),first=f.openRemote()
 assert.equal(f.state.opening,true)
 await f.openRemote()
 assert.equal(f.requests.length,1)
 const request=f.requests[0]
 assert.equal(request.path,'/api/remote/open')
 assert.deepEqual(request.options,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal:'mock-signal'})
 assert.deepEqual(f.timeouts,[35000])
 request.resolve({ok:true})
 await first
 assert.deepEqual(f.state,{opening:false,error:''})
 assert.equal(f.pending.current,false)
})

for(const failure of ['http','network','timeout'])test(failure+' failure restores the button and permits an explicit retry',async()=>{
 const f=fixture(),first=f.openRemote()
 if(failure==='http')f.requests[0].resolve({ok:false,status:503})
 else f.requests[0].reject(Error(failure))
 await first
 assert.deepEqual(f.state,{opening:false,error:'打开失败，请重试'})
 assert.equal(f.pending.current,false)
 const retry=f.openRemote()
 assert.deepEqual(f.state,{opening:true,error:''})
 assert.equal(f.requests.length,2)
 f.requests[1].resolve({ok:true})
 await retry
 assert.deepEqual(f.state,{opening:false,error:''})
})
