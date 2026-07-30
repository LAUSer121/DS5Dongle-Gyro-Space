const fs=require('fs'); let js=fs.readFileSync('/tmp/portal.js','utf8');
const stop=js.indexOf('\nrender();\nportalSelfCheck();'); js=js.slice(0,stop>0?stop:js.length);
global.setInterval=()=>0;global.setTimeout=()=>0;global.requestAnimationFrame=()=>0;
global.window={setInterval:()=>0,setTimeout:()=>0};
global.document={getElementById:()=>null,createElement:()=>({style:{},classList:{add(){}}}),body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
const M=new Function('window','document','navigator','location', js+` return { ceValidateStates };`)
 (global.window,global.document,global.navigator,global.location);
const RES=[0x21,0x80,0x03,0,0,0,0,0,0,0,0], WB=[0x25,0x10,0x01,0x03,0,0,0,0,0,0,0];
const VA =[0x26,0,3,0,0,0,0x2d,0,0,0x0a,0], VB=[0x26,0,3,0,0,0,0x09,0,0,0x2d,0];

let v = M.ceValidateStates([{bytes:RES,n:'Res#1'},{bytes:WB,n:'WB'},{bytes:RES,n:'Res#3'}], e=>e.bytes);
console.log("mechanical [Res#1, WB, Res#3]  -> kept:", v.kept.map(e=>e.n).join(', '), "| dropped:", v.dropped.length);

v = M.ceValidateStates([{bytes:WB,n:'WB'},{bytes:RES,n:'Res#2'},{bytes:RES,n:'Res#3'}], e=>e.bytes);
console.log("mechanical [WB, Res#2, Res#3]  -> kept:", v.kept.map(e=>e.n).join(', '), "| dropped:", v.dropped.length);

v = M.ceValidateStates([{bytes:VA,duration_ms:120,n:'A/120'},{bytes:VB,duration_ms:900,n:'B/900'},
                        {bytes:VA,duration_ms:500,n:'A/500'}], e=>e.bytes);
console.log("vibration  [A/120, B/900, A/500] -> kept:", v.kept.map(e=>e.n).join(', '), "| dropped:", v.dropped.length);
console.log(v.kept.length===3 ? "   rhythm preserved (A/500 is a different beat, not a repeat)" : "   RHYTHM LOST");
