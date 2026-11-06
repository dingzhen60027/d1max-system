import {useRef,useState} from 'react'
import {Gamepad2} from 'lucide-react'
import {Button} from '@/components/ui/button'

export default function RemoteLaunchButton() {
 const [opening,setOpening]=useState(false),[error,setError]=useState('')
 const pending=useRef(false)
 const openRemote=async()=>{
  if(pending.current)return
  pending.current=true;setOpening(true);setError('')
  try{
   const response=await fetch('/api/remote/open',{
    method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal:AbortSignal.timeout(35000),
   })
   if(!response.ok)throw Error('Remote launch failed')
  }catch{
   setError('打开失败，请重试')
  }finally{
   pending.current=false;setOpening(false)
  }
 }
 return <div className="remote-launch">
  <Button variant="outline" size="sm" onClick={openRemote} disabled={opening} aria-busy={opening}><Gamepad2/>{opening?'打开中…':'遥控器'}</Button>
  {error&&<span className="remote-launch-error" role="alert">{error}</span>}
 </div>
}
