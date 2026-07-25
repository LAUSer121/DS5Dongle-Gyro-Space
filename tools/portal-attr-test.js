// Regression guard for the class of bug that broke every dropdown in 1.14.6:
// a quote embedded in a generated event attribute silently truncates the handler.
const fs = require('fs');
let js = fs.readFileSync('/tmp/portal.js','utf8');
const stop = js.indexOf('\nrender();\nportalSelfCheck();');
js = js.slice(0, stop > 0 ? stop : js.length);
global.setInterval=()=>0; global.setTimeout=()=>0; global.requestAnimationFrame=()=>0;
global.window = {setInterval:()=>0,setTimeout:()=>0};
let captured='';
const appEl={set innerHTML(v){captured=v;},get innerHTML(){return captured;}};
global.document={getElementById:id=>id==='app'?appEl:null,createElement:()=>({style:{},classList:{add(){}}}),
  body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
const fn=new Function('window','document','navigator','location', js + `
  return { TABS, render, setDev:d=>{device=d;}, setTab:t=>{window._tab=t;} };`);
const M=fn(global.window,global.document,global.navigator,global.location);
M.setDev({productId:0x0ce6,opened:true});
let bad=0, checked=0;
for(const t of M.TABS){
  M.setTab(t.id); M.render();
  // every on*="..." must contain no stray double quote and must be balanced JS-ish
  for(const m of captured.matchAll(/\son(change|click|input)="([^"]*)"/g)){
    checked++;
    const body=m[2];
    if(body.includes('"')){ console.log('  QUOTE IN HANDLER:', body.slice(0,70)); bad++; }
    const par=(body.match(/\(/g)||[]).length-(body.match(/\)/g)||[]).length;
    if(par!==0){ console.log('  UNBALANCED HANDLER:', body.slice(0,70)); bad++; }
  }
  // a truncated attribute leaves a bare `if(` or trailing operator at the end
  for(const m of captured.matchAll(/\son(change|click)="([^"]*)$/gm)){ console.log('  UNTERMINATED:', m[2].slice(0,60)); bad++; }
}
console.log(`event handlers checked: ${checked} | problems: ${bad}`);
console.log(bad ? 'ATTRIBUTE TEST FAILED' : 'ATTRIBUTE TEST OK');
