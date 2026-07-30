const fs=require('fs'); let js=fs.readFileSync('/tmp/portal.js','utf8');
const stop=js.indexOf('\nrender();\nportalSelfCheck();'); js=js.slice(0,stop>0?stop:js.length);
global.setInterval=()=>0;global.setTimeout=()=>0;global.requestAnimationFrame=()=>0;
global.window={setInterval:()=>0,setTimeout:()=>0};
global.document={getElementById:()=>null,createElement:()=>({style:{},classList:{add(){}}}),body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
const M=new Function('window','document','navigator','location', js+` return { ceDescribeState };`)
 (global.window,global.document,global.navigator,global.location);
const hex=s=>s.split(' ').map(x=>parseInt(x,16)).concat(Array(11).fill(0)).slice(0,11);
[["25 0c 00 00","YOUR wall — force byte 0"],
 ["25 0c 00 07","same wall at full force"],
 ["25 10 01 03","your other wall"],
 ["21 80 03","captured resistance, zero force field"],
 ["21 0c 00 c0 0f","GoW2018 resistance"]
].forEach(([h,l])=>console.log(`  ${M.ceDescribeState(hex(h)).padEnd(58)} <- ${l}`));
