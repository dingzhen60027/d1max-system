import test from 'node:test'
import assert from 'node:assert/strict'
import {navigationControls,navigationReason} from '../src/navigation-status.mjs'
const version='grid-'+'a'.repeat(24)
const current={phase:'running',session_id:'a'.repeat(32),version_id:version,mode:'live',enable_motion:true,health:{runtime_available:true,gate:{sdk_session:'b'.repeat(16),armed:false,arm_ready:true,reason:'not_armed'}}}
test('live motion remains explicit, and Web has no goal or cancel controls',()=>{
 const c=navigationControls(current,version);assert(c.canArm);assert(c.canDisarm)
 assert(!('canGoal' in c));assert(!('canCancel' in c))
 assert(!navigationControls({...current,health:{...current.health,gate:{...current.health.gate,armed:true}}},version).canArm)
})
test('stale state, missing capability and map mismatch fail closed',()=>{
 for(const value of [{...current,phase:'stopped'},{...current,session_id:null},{...current,health:{}},{...current,enable_motion:false},{...current,health:{...current.health,gate:{...current.health.gate,sdk_session:null}}}])assert(!navigationControls(value,version).canArm)
 assert(!navigationControls(current,'grid-'+'b'.repeat(24)).canArm)
})
test('simulation never arms SDK, and active RViz goals block rearming',()=>{
 const sim={...current,mode:'sim',enable_motion:false};assert(!navigationControls(sim,version).canArm);assert(!navigationControls(sim,version).canDisarm)
 for(const status of [1,2,3])assert(!navigationControls({...current,health:{...current.health,action:{status}}},version).canArm)
})
test('unverified calibration and missing sensor reasons are readable',()=>{
 assert.equal(navigationReason('localization_not_verified'),'外参或时间同步尚未核验')
 assert.equal(navigationReason('future_reason'),'future_reason')
})
