const fs=require('fs'); let js=fs.readFileSync('/tmp/portal.js','utf8');
const stop=js.indexOf('\nrender();\nportalSelfCheck();'); js=js.slice(0,stop>0?stop:js.length);
global.setInterval=()=>0;global.setTimeout=()=>0;global.requestAnimationFrame=()=>0;
global.window={setInterval:()=>0,setTimeout:()=>0};
global.document={getElementById:()=>null,createElement:()=>({style:{},classList:{add(){}}}),body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
const M=new Function('window','document','navigator','location', js+`
 return { ceValidateStates };`)(global.window,global.document,global.navigator,global.location);
const RES=[0x21,0x80,0x03,0,0,0,0,0,0,0,0];
const WB =[0x25,0x10,0x01,0x03,0,0,0,0,0,0,0];
const WB2=[0x25,0x24,0x00,0x07,0,0,0,0,0,0,0];   // starts zone 2
let r = M.ceValidateStates([RES,WB,RES], b=>b);
console.log("user's exact 3 picks -> kept:", r.kept.length, "dropped:", r.dropped.length, "clash:", r.clash);
r = M.ceValidateStates([RES,WB], b=>b);
console.log("the correct 2 picks   -> kept:", r.kept.length, "dropped:", r.dropped.length, "clash:", r.clash);
r = M.ceValidateStates([WB2,WB], b=>b);
console.log("two DIFFERENT walls, same start? ->", "clash:", r.clash, "(zone 2 vs 4, expect null)");
const A=[0x25,0x24,0x00,0x07,0,0,0,0,0,0,0], B=[0x25,0x24,0x00,0x03,0,0,0,0,0,0,0];
r = M.ceValidateStates([A,B], b=>b);
console.log("distinct bytes, SAME start zone 2 ->", "clash:", r.clash, "(expect 2)");
const ok = M.ceValidateStates([RES,WB,RES],b=>b).dropped.length===1
        && M.ceValidateStates([RES,WB,RES],b=>b).clash===null
        && M.ceValidateStates([A,B],b=>b).clash===2;
console.log(ok ? "VALIDATOR OK" : "VALIDATOR FAILED");
