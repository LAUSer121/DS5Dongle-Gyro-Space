const fs=require('fs'); let js=fs.readFileSync('/tmp/portal.js','utf8');
const stop=js.indexOf('\nrender();\nportalSelfCheck();'); js=js.slice(0,stop>0?stop:js.length);
global.setInterval=()=>0;global.setTimeout=()=>0;global.requestAnimationFrame=()=>0;
global.window={setInterval:()=>0,setTimeout:()=>0};
global.document={getElementById:()=>null,createElement:()=>({style:{},classList:{add(){}}}),body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
const M=new Function('window','document','navigator','location', js+`
 return { ceDescribeState, ceActiveWindows, cebBuildBytes, ceZonesOfBytes };`)
 (global.window,global.document,global.navigator,global.location);
const hex=s=>s.split(' ').map(x=>parseInt(x,16)).concat(Array(11).fill(0)).slice(0,11);
console.log("=== real captured bytes from this session ===");
[["25 24 00 07","R&C ready wall"],["25 20 01 07","R&C after break 1"],["21 00 02","R&C bottom hold"],
 ["21 80 03","user's GoW/other resistance"],["25 10 01 03","user's weapon break"],
 ["21 0c 00 c0 0f","GoW2018 resistance"],["22 a0 00 3f","GoW2018 bow"],
 ["22 88 00 19","Gas.json bow 1"],["26 00 03 00 00 00 09 00 00 2d","GoW spear vibration"]
].forEach(([h,label])=>console.log(`  ${M.ceDescribeState(hex(h)).padEnd(52)} <- ${label}`));

console.log("\n=== round-trip: build in the builder, read it back ===");
const st={type:'break',start:2,end:5,strength:100};
const b=M.cebBuildBytes(st);
console.log("  built  wall 2->5 @100%  ->", b.slice(0,4).map(x=>x.toString(16).padStart(2,'0')).join(' '));
console.log("  reader says:", M.ceDescribeState(b));

console.log("\n=== active windows: the user's 2-state set (wall + resistance) ===");
const list=[hex("25 10 01 03"), hex("21 80 03")];
const w=M.ceActiveWindows(list);
list.forEach((b,i)=>{
  const z=M.ceZonesOfBytes(b);
  console.log(`  ${M.ceDescribeState(b).padEnd(46)} stage ${w[i].order}, active ${w[i].from}-${w[i].to}  (own span ends ${Math.round(z.slice(-1)[0]*25.5)})`);
});
console.log("  -> matches the hand analysis: wall handed over at 166, breaks at 204");
