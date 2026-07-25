const fs = require('fs');
let js = fs.readFileSync('/tmp/portal.js','utf8');
// keep every declaration, stop before the startup calls
const stop = js.indexOf('\nrender();\nportalSelfCheck();');
js = js.slice(0, stop > 0 ? stop : js.length);
global.setInterval=()=>0; global.setTimeout=()=>0; global.requestAnimationFrame=()=>0;
global.window = {setInterval:()=>0,setTimeout:()=>0}; global.location = { search: '' };
global.document = { getElementById: () => null, createElement: () => ({style:{},classList:{add(){}}}),
                    body:{prepend(){}}, addEventListener(){}, querySelector: ()=>null };
global.navigator = { hid: { getDevices: async()=>[], addEventListener(){} } };
const fn = new Function('window','document','navigator','location', js + `
  return { SECTIONS, FIELDS, TABS, SECTION_TAB, portalSelfCheck };`);
const M = fn(global.window, global.document, global.navigator, global.location);
const tabIds = new Set(M.TABS.map(t=>t.id));
let total=0, unreachable=[], perTab={};
for (const sec of M.SECTIONS) for (const f of sec.fields){
  total++; const key=f[1];
  const tab = M.SECTION_TAB[sec.title];
  if(!tabIds.has(tab)) unreachable.push(`${key} -> ${tab}`);
  perTab[tab]=(perTab[tab]||0)+1;
}
console.log('fields in SECTIONS:', total, '| FIELDS length:', M.FIELDS.length);
console.log('per tab:', JSON.stringify(perTab, null, 0));
console.log('unreachable fields:', unreachable.length ? unreachable : 'none');
const byId={}; let clash=0;
for(const [fid,key] of M.FIELDS){ if(byId[fid]&&byId[fid]!==key){console.log('  ID CLASH 0x'+fid.toString(16), byId[fid], key); clash++;} byId[fid]=key; }
console.log('field-id clashes:', clash);
console.log('self-check passes:', M.portalSelfCheck());
console.log(total===M.FIELDS.length && !unreachable.length && !clash ? 'COVERAGE OK' : 'COVERAGE FAILED');
