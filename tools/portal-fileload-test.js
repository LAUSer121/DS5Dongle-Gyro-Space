const fs=require('fs'); let js=fs.readFileSync('/tmp/portal.js','utf8');
const stop=js.indexOf('\nrender();\nportalSelfCheck();'); js=js.slice(0,stop>0?stop:js.length);
global.setInterval=()=>0;global.setTimeout=()=>0;global.requestAnimationFrame=()=>0;
global.window={setInterval:()=>0,setTimeout:()=>0};
global.document={getElementById:()=>null,createElement:()=>({style:{},classList:{add(){}}}),body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
const M=new Function('window','document','navigator','location', js+` return { ceValidateStates };`)
 (global.window,global.document,global.navigator,global.location);
const file = JSON.parse(fs.readFileSync('/mnt/user-data/uploads/R2_3x.json','utf8'));
console.log("R2_3x.json declares state_count:", file.state_count, "| states in file:", file.states.length);
const v = M.ceValidateStates(file.states, st => st.bytes);
console.log("  kept:", v.kept.length, "| dropped as exact repeats:", v.dropped.length, "| zone clash:", v.clash);
console.log("  would load:", v.kept.map(s=>s.name).join(' + '));
console.log(v.kept.length===2 && v.dropped.length===1 && v.clash===null
  ? "\nFILE GUARD OK — loads 2 states, no clicking (before this fix it loaded 3 and clicked)"
  : "\nUNEXPECTED");
