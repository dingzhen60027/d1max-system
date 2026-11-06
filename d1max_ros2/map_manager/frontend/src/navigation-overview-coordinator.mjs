// One in-flight overview and at most one coalesced follow-up. A lifecycle
// write invalidates older reads; response arrival order is not state order.
export function createNavigationOverviewCoordinator({read,onValue,onError}) {
 let generation=0,writing=false,inflight=null,queued=null
 const discardQueued=()=>{if(queued){queued.resolve(false);queued=null}}
 const launch=epoch=>{
  const job={epoch,promise:null}
  inflight=job
  job.promise=Promise.resolve().then(read).then(value=>{
   if(epoch!==generation||writing)return false
   onValue(value);return true
  },error=>{
   if(epoch===generation&&!writing)onError(error)
   return false
  }).finally(()=>{
   if(inflight!==job)return
   inflight=null
   const next=queued;queued=null
   if(next){
    if(next.epoch===generation&&!writing)launch(next.epoch).then(next.resolve)
    else next.resolve(false)
   }
  })
  return job.promise
 }
 const refresh=()=>{
  if(writing)return Promise.resolve(false)
  if(!inflight)return launch(generation)
  if(inflight.epoch===generation)return inflight.promise
  if(!queued){
   let resolve
   const promise=new Promise(done=>{resolve=done})
   queued={epoch:generation,promise,resolve}
  }
  return queued.promise
 }
 return {
  refresh,
  beginWrite(){generation++;writing=true;discardQueued()},
  finishWrite({refresh:shouldRefresh=true}={}) {
   writing=false
   return shouldRefresh?refresh():Promise.resolve(false)
  },
  invalidate(){generation++;discardQueued()},
 }
}
