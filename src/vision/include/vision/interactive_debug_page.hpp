#pragma once

namespace vision {
// Canvas is drawn only after a complete JPEG is decoded; input carries the ID
// of exactly that canvas. No input replay on reconnect or failed POST.
inline constexpr char kInteractiveDebugPage[] = R"HTML(<!doctype html>
<html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera test</title><style>
body{margin:0;background:#111;color:#ddd;font:14px sans-serif}
canvas{display:block;max-width:100%;height:auto;margin:auto;outline:none;touch-action:manipulation}
#status{padding:8px;text-align:center}button{margin:3px;padding:6px 12px}
</style><canvas id="view" tabindex="0"></canvas>
<div id="status">Connecting...</div><div style="text-align:center">
<button data-key="32">Space / live</button><button data-key="3">Ctrl+C / live</button>
<button data-key="13">Enter</button><button data-key="27">Esc</button></div>
<script>
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');
const status=document.getElementById('status');let frameId='0',pending=false,queue=[];
const decoder=new TextDecoder();
async function input(text){
  if(frameId==='0'||queue.length>=32)return;queue.push(text);if(pending)return;pending=true;
  try{while(queue.length){const next=queue.shift();
    const r=await fetch('/input',{method:'POST',headers:{'Content-Type':'text/plain'},body:next});
    if(!r.ok){queue=[];status.textContent='Input was not accepted. Wait for the current screen.';break;}}
  }catch(e){queue=[];status.textContent='Connection lost; input was not retried.';}
  finally{pending=false;}
}
canvas.addEventListener('mousedown',e=>{
  if(e.button!==0&&e.button!==2)return;e.preventDefault();canvas.focus();
  const r=canvas.getBoundingClientRect();
  const x=Math.floor((e.clientX-r.left)*canvas.width/r.width);
  const y=Math.floor((e.clientY-r.top)*canvas.height/r.height);
  if(x>=0&&y>=0&&x<canvas.width&&y<canvas.height)input(`click ${frameId} ${e.button} ${x} ${y}`);
});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
function sendKey(code){input(`key ${frameId} ${code}`);}
document.querySelectorAll('[data-key]').forEach(b=>b.onclick=()=>sendKey(Number(b.dataset.key)));
document.addEventListener('keydown',e=>{
  if(e.repeat||e.altKey||e.metaKey)return;
  let code=e.ctrlKey&&e.key.toLowerCase()==='c'?3:
    ({Enter:13,Escape:27,Backspace:8,Delete:127,' ':32})[e.key];
  if(code===undefined&&!e.ctrlKey&&e.key.length===1)code=e.key.charCodeAt(0);
  if(code!==undefined&&code>=0&&code<=127){e.preventDefault();sendKey(code);}
});
async function stream(){
  frameId='0';const r=await fetch('/stream.mjpg',{cache:'no-store'});
  if(!r.ok||!r.body)throw Error('stream');const reader=r.body.getReader();
  let buffer=new Uint8Array(0);
  async function more(){const part=await reader.read();if(part.done)throw Error('closed');
    if(buffer.length+part.value.length>4194304)throw Error('frame too large');
    const next=new Uint8Array(buffer.length+part.value.length);next.set(buffer);next.set(part.value,buffer.length);buffer=next;}
  try{while(true){
    let end=-1;
    while(end<0){for(let i=0;i+3<buffer.length;i++)if(buffer[i]===13&&buffer[i+1]===10&&buffer[i+2]===13&&buffer[i+3]===10){end=i;break;}
      if(end<0){if(buffer.length>4096)throw Error('header');await more();}}
    const header=decoder.decode(buffer.subarray(0,end));
    const length=Number(/Content-Length:\s*(\d+)/i.exec(header)?.[1]);
    const id=/X-Frame-Id:\s*(\d+)/i.exec(header)?.[1];
    if(!Number.isSafeInteger(length)||length<=0||length>4000000||!id)throw Error('invalid frame');
    buffer=buffer.subarray(end+4);
    while(buffer.length<length+2)await more();
    const bitmap=await createImageBitmap(new Blob([buffer.slice(0,length)],{type:'image/jpeg'}));
    buffer=buffer.slice(length+2);
    if(canvas.width!==bitmap.width||canvas.height!==bitmap.height){canvas.width=bitmap.width;canvas.height=bitmap.height;}
    ctx.drawImage(bitmap,0,0);bitmap.close();frameId=id;status.textContent='Connected — click the image or use keys.';
  }}finally{frameId='0';queue=[];await reader.cancel().catch(()=>{});}
}
(async()=>{while(true){try{await stream();}catch(e){frameId='0';status.textContent='Disconnected — reconnecting...';}
  await new Promise(resolve=>setTimeout(resolve,1000));}})();
</script></html>)HTML";
}  // namespace vision
