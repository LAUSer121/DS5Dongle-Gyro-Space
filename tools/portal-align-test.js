const fs=require('fs');
let js=fs.readFileSync('/tmp/portal.js','utf8');
const stop=js.indexOf('\nrender();\nportalSelfCheck();'); js=js.slice(0,stop>0?stop:js.length);
global.setInterval=()=>0; global.setTimeout=()=>0; global.requestAnimationFrame=()=>0;
global.window={setInterval:()=>0,setTimeout:()=>0};
let cap=''; const app={set innerHTML(v){cap=v;},get innerHTML(){return cap;}};
global.document={getElementById:id=>id==='app'?app:null,createElement:()=>({style:{},classList:{add(){}}}),
  body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
const M=new Function('window','document','navigator','location', js + `
  return { SECTIONS, render, triggerPairHTML, trigLogical, SHARED_TRIGGER_KEYS,
           R2_TITLE, L2_TITLE, setDev:d=>{device=d;}, setTab:t=>{window._tab=t;} };`)
  (global.window,global.document,global.navigator,global.location);
M.setDev({productId:0x0ce6,opened:true}); M.setTab('triggers'); M.render();

const h = M.triggerPairHTML();
const cells = (h.match(/class="tcell"/g)||[]).length;
console.log('grid cells:', cells, '| rows:', cells/2, '| even:', cells%2===0);

// pair alignment: for each row, left and right must share a logical key
const secOf = t => (M.SECTIONS.find(s=>s.title===t)||{fields:[]}).fields;
const r2 = secOf(M.R2_TITLE).filter(f=>!M.SHARED_TRIGGER_KEYS.has(f[1]));
const l2 = secOf(M.L2_TITLE).filter(f=>!M.SHARED_TRIGGER_KEYS.has(f[1]));
let bad=0;
for(let i=0;i<Math.max(r2.length,l2.length);i++){
  const a=r2[i]?M.trigLogical(r2[i][1]):null, b=l2[i]?M.trigLogical(l2[i][1]):null;
  if(a!==b){ console.log(`  ROW ${i} MISALIGNED: ${a} vs ${b}`); bad++; }
}
console.log(`R2 rows: ${r2.length} | L2 rows: ${l2.length} | misaligned: ${bad}`);
const shared=(h.match(/class="tshared"/g)||[]).length;
console.log('shared full-width block present:', shared===1);
// no field lost
const keys=new Set([...h.matchAll(/config\['([a-z0-9_]+)'\]/g)].map(m=>m[1]));
console.log('distinct fields in the pair card:', keys.size, '(expect 35: 17+17+1 shared)');
console.log(bad===0 && cells%2===0 && shared===1 && keys.size===35 ? 'ALIGNMENT OK' : 'ALIGNMENT FAILED');
