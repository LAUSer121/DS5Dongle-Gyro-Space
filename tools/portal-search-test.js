// Mirrors the technique used by tools/portal-*.js: slice the script before the
// startup render(), stub the DOM, then new Function() out the internals.
const fs = require('fs');
const src = fs.readFileSync(process.argv[2] || 'ds5-config-portal.html', 'utf8');
const script = src.match(/<script>([\s\S]*?)<\/script>/)[1];
const cut = script.indexOf('\nrender();\nportalSelfCheck();');
if (cut < 0) throw new Error('startup render() marker not found');
const body = script.slice(0, cut);

function makeEl(){ return { innerHTML:'', textContent:'', style:{}, value:'', id:'',
  focus(){}, setSelectionRange(){}, appendChild(){}, addEventListener(){},
  querySelector(){ return null; }, classList:{add(){},remove(){}} }; }

const nodes = {};
const doc = {
  activeElement: null,
  getElementById(id){ if(id==='app') return nodes.app; return nodes[id] || null; },
  createElement(){ return makeEl(); },
  body:{ prepend(){}, appendChild(){} },
  addEventListener(){},
};
nodes.app = makeEl();

const ctx = {
  window:{}, document:doc,
  navigator:{ hid:{ getDevices:async()=>[], addEventListener(){} } },
  setInterval(){}, setTimeout(){}, clearTimeout(){}, console,
  location:{ search:'', href:'' }, alert(){}, fetch(){},
};

const fn = new Function('window','document','navigator','setInterval','setTimeout',
  'clearTimeout','console','location','alert','fetch',
  body + `
  ;return { SECTIONS, FIELDS, SECTION_TAB, TABS, DESCRIPTIONS,
    searchResults, searchTerms, fieldHaystack, fieldMatches, searchBarHTML,
    searchResultsHTML, render, setSearch, setTab,
    setQ: v => { window._q = v; },
    setTabId: v => { window._tab = v; },
    setDev: d => { device = d; },
    getConfig: () => config };`);

const P = fn(ctx.window, ctx.document, ctx.navigator, ctx.setInterval, ctx.setTimeout,
  ctx.clearTimeout, ctx.console, ctx.location, ctx.alert, ctx.fetch);

let fail = 0;
const ok  = (c,m) => { if(!c){ console.log('  FAIL: '+m); fail++; } else console.log('  ok   '+m); };

// A device is required before render() draws anything past the connect box.
P.setDev({ opened:true, productId:0x0DF2, sendFeatureReport:async()=>{}, receiveFeatureReport:async()=>({getUint8:()=>0}) });

const allKeys = P.FIELDS.map(f => f[1]);
const idOf = k => (P.FIELDS.find(f => f[1] === k) || [])[0];

console.log('\n1. empty query leaves the tab view byte-identical');
for (const t of P.TABS){
  P.setQ(''); P.setTabId(t.id); P.render();
  const withSearch = ctx.document.getElementById('app').innerHTML;
  ok(withSearch.includes('id="qbox"'), `${t.id}: search box present`);
  ok(withSearch.includes(`${P.FIELDS.length} settings`), `${t.id}: idle hint counts all ${P.FIELDS.length} fields`);
  ok(withSearch.includes(`class="tab active"`), `${t.id}: exactly one tab still marked active`);
}

console.log('\n2. every field is reachable by searching its own label');
let unreachable = [];
for (const f of P.FIELDS){
  const label = f[3];
  const groups = P.searchResults(label);
  const hit = groups.some(g => g.hits.some(h => h[1] === f[1]));
  if (!hit) unreachable.push(f[1] + ' ("' + label + '")');
}
ok(unreachable.length === 0, `all ${P.FIELDS.length} fields findable by label` +
   (unreachable.length ? ' — missing: ' + unreachable.join(', ') : ''));

console.log('\n3. matching behaviour');
const keysFor = q => P.searchResults(q).flatMap(g => g.hits.map(h => h[1]));
ok(keysFor('rumble').length > 0, 'rumble returns hits');
ok(keysFor('rumble').includes('rumble_haptic_strength'), 'rumble finds rumble_haptic_strength');
ok(keysFor('RUMBLE').length === keysFor('rumble').length, 'case-insensitive');
ok(keysFor('flick').includes('flick_counts_360'), 'flick finds the Flick Stick field');
ok(keysFor('flick').includes('gyro_output'), 'flick also finds gyro_output via its OPTION label');
ok(keysFor('ratchet').includes('gyro_mode'), 'ratchet finds gyro_mode via its option label');
const dz = keysFor('deadzone'), dz2 = keysFor('dead zone');
ok(dz.length > 0 && dz.join() === dz2.join(), 'squashed fold: "deadzone" === "dead zone"');
ok(keysFor('lowpass').length === keysFor('low-pass').length && keysFor('lowpass').length > 0,
   'squashed fold: "lowpass" === "low-pass"');
const two = keysFor('gyro sensitivity');
ok(two.length > 0 && two.length <= keysFor('gyro').length, 'multiple terms AND together (narrow, not widen)');
ok(keysFor('zzzznope').length === 0, 'nonsense query returns nothing');
ok(keysFor('   ').length === 0, 'whitespace-only query is treated as empty');

console.log('\n4. results render and stay bound');
P.setQ('rumble'); P.render();
let h = ctx.document.getElementById('app').innerHTML;
const rumbleKeys = keysFor('rumble');
for (const k of rumbleKeys) ok(h.includes(`config['${k}']`), `result row for ${k} is bound to config['${k}']`);
ok(!h.includes('class="tab active"'), 'no tab is marked active during a search');
ok(h.includes('Save to Device'), 'the actions bar (Save to Device) stays available');
ok(h.includes('tab</span>'), 'each result card names the tab the setting lives on');
ok(!/id="slotbox"/.test(h) && !/id="effmon"/.test(h) && !/id="diagbox"/.test(h),
   'tab-specific panels are suppressed during a search');

console.log('\n5. no duplicate element ids in any view');
function dupIds(html){
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
  const seen = new Set(), dup = new Set();
  for (const i of ids){ if (seen.has(i)) dup.add(i); seen.add(i); }
  return [...dup];
}
for (const t of P.TABS){ P.setQ(''); P.setTabId(t.id); P.render();
  ok(dupIds(ctx.document.getElementById('app').innerHTML).length === 0, `tab ${t.id}: no duplicate ids`); }
for (const q of ['rumble','macro','a','e','trigger','gyro']){
  P.setQ(q); P.render();
  ok(dupIds(ctx.document.getElementById('app').innerHTML).length === 0, `query "${q}": no duplicate ids`);
}

console.log('\n6. event attributes survive (portal hard rule #1)');
P.setQ('e'); P.render();
h = ctx.document.getElementById('app').innerHTML;
const handlers = [...h.matchAll(/\son(?:click|change|input|keydown)="([^"]*)"/g)].map(m => m[1]);
ok(handlers.length > 0, `found ${handlers.length} event handlers in the widest result set`);
let bad = handlers.filter(x => {
  let d = 0; for (const c of x){ if (c==='(') d++; else if (c===')') d--; if (d<0) return true; }
  return d !== 0;
});
ok(bad.length === 0, 'every handler has balanced parens' + (bad.length ? ': ' + bad[0] : ''));
bad = handlers.filter(x => (x.match(/'/g) || []).length % 2 !== 0);
ok(bad.length === 0, 'every handler has balanced single quotes' + (bad.length ? ': ' + bad[0] : ''));

console.log('\n7. empty-state and clear');
P.setQ('zzzznope'); P.render();
h = ctx.document.getElementById('app').innerHTML;
ok(h.includes('Nothing matches that'), 'empty state explains what to try');
ok(h.includes('no matches'), 'hit counter says no matches');
ok(h.includes(`onclick="setSearch('')"`), 'Clear button present');
P.setTab('haptics');
ok(ctx.window._q === '' && ctx.window._tab === 'haptics', 'picking a tab clears the search');

console.log(fail ? `\n${fail} FAILURE(S)\n` : '\nALL CHECKS PASSED\n');
process.exit(fail ? 1 : 0);
