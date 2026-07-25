const fs = require('fs');
let js = fs.readFileSync('/tmp/portal.js','utf8');
const stop = js.indexOf('\nrender();\nportalSelfCheck();');
js = js.slice(0, stop > 0 ? stop : js.length);
global.setInterval=()=>0; global.setTimeout=()=>0; global.requestAnimationFrame=()=>0;
global.window = {setInterval:()=>0,setTimeout:()=>0};
let captured = '';
const appEl = { set innerHTML(v){ captured = v; }, get innerHTML(){ return captured; } };
global.document = { getElementById: id => id==='app' ? appEl : null,
  createElement: () => ({style:{},classList:{add(){}}}), body:{prepend(){}},
  addEventListener(){}, querySelector: ()=>null };
global.navigator = { hid: { getDevices: async()=>[], addEventListener(){} } };
const fn = new Function('window','document','navigator','location', js + `
  return { SECTIONS, FIELDS, TABS, render, setDev: d => { device = d; },
           setTab: t => { window._tab = t; }, refreshSlots: () => {} };`);
const M = fn(global.window, global.document, global.navigator, global.location);
M.setDev({ productId: 0x0ce6, opened:true });          // pretend connected
const seen = new Set(); const report = [];
for (const t of M.TABS){
  M.setTab(t.id);
  try { M.render(); } catch(e){ console.log(`  ${t.id}: RENDER THREW ${e.message}`); continue; }
  const html = captured;
  const keys = [...html.matchAll(/config\['([a-z0-9_]+)'\]/g)].map(m=>m[1]);
  keys.forEach(k=>seen.add(k));
  const cards = [...html.matchAll(/class="card-title">([^<]{0,44})/g)].map(m=>m[1].trim());
  report.push(`  ${t.label.padEnd(15)} ${String(new Set(keys).size).padStart(2)} fields | cards: ${cards.join(' · ') || '(none)'}`);
  // the tab bar must always be present and mark exactly one active
  const active = (html.match(/class="tab active"/g)||[]).length;
  if (active !== 1) console.log(`  !! ${t.id}: ${active} active tabs`);
}
report.forEach(r=>console.log(r));
const all = new Set(M.FIELDS.map(f=>f[1]));
const missing = [...all].filter(k=>!seen.has(k));
console.log('\nfields rendered across all tabs:', seen.size, 'of', all.size);
console.log('never rendered:', missing.length ? missing : 'none');
console.log(missing.length ? 'RENDER TEST FAILED' : 'RENDER TEST OK');
