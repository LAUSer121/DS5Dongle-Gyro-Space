// The macro panel is drawn by macroRender() into #macrobox at runtime, so
// portal-attr-test.js - which only inspects render()'s output into #app - never
// sees a single one of its generated handlers. This covers that gap with the
// same checks, plus the encoding the picker produces.
//
// The bug class being guarded: a quote inside a generated on*="..." attribute
// closes it early and silently truncates the handler. That broke all 37 selects
// in 1.14.7 and was invisible until a dropdown was clicked.
const fs = require('fs');
let js = fs.readFileSync('/tmp/portal.js', 'utf8');
const stop = js.indexOf('\nrender();\nportalSelfCheck();');
js = js.slice(0, stop > 0 ? stop : js.length);

global.setInterval = () => 0; global.setTimeout = () => 0; global.requestAnimationFrame = () => 0;
global.window = { setInterval: () => 0, setTimeout: () => 0 };
let appHtml = '', boxHtml = '';
const appEl = { set innerHTML(v){ appHtml = v; }, get innerHTML(){ return appHtml; } };
const boxEl = { set innerHTML(v){ boxHtml = v; }, get innerHTML(){ return boxHtml; },
                set textContent(v){ boxHtml = v; }, get textContent(){ return boxHtml; } };
global.document = {
  getElementById: id => id === 'app' ? appEl : (id === 'macrobox' ? boxEl : null),
  createElement: () => ({ style: {}, classList: { add(){} } }),
  body: { prepend(){} }, addEventListener(){}, querySelector: () => null
};
global.navigator = { hid: { getDevices: async () => [], addEventListener(){} } };

const fn = new Function('window', 'document', 'navigator', 'location', js + `
  return { macroRender, macroPickApply, macroPickOpen, macroPickMod, macroPickKey,
           macroChordName, macroGestureName, macroComboName, MACRO_COUNT,
           setRows: r => { window._macroRows = r; },
           getRows: () => window._macroRows,
           setPick: p => { window._macroPick = p; },
           setSnap: x => { savedSnapshot = x; },
           macroTableToJson, macroTableFromJson,
           macroSetBit, getCfg: () => config,
           setCfg: c => { config = c; },
           setDev: d => { device = d; } };`);
const M = fn(global.window, global.document, global.navigator, global.location);

let bad = 0, checked = 0, tests = 0;
function ok(cond, what){ tests++; console.log('  ' + (cond ? 'ok  ' : 'FAIL') + '  ' + what); if (!cond) bad++; }

function scanHandlers(html, label){
  for (const m of html.matchAll(/\son(change|click|input)="([^"]*)"/g)){
    checked++;
    const body = m[2];
    if (body.includes('"')){ console.log('  QUOTE IN HANDLER (' + label + '):', body.slice(0,70)); bad++; }
    const par = (body.match(/\(/g)||[]).length - (body.match(/\)/g)||[]).length;
    if (par !== 0){ console.log('  UNBALANCED (' + label + '):', body.slice(0,70)); bad++; }
  }
  for (const m of html.matchAll(/\son(change|click|input)="([^"]*)$/gm)){
    console.log('  UNTERMINATED (' + label + '):', m[2].slice(0,60)); bad++;
  }
}

const blank = () => ({chord:0,gesture:0,flags:0,hold_cs:0,keys:[0,0,0,0],rel_order:0,label:'',present:false});
M.setDev({ productId: 0x0ce6, opened: true });
M.setCfg({ macro_disable: 0xFFFFFFFE >>> 0 });

// A row carrying hostile text in the NAME - the field a user types into freely,
// and the only place attacker-ish characters can reach an attribute.
const rows = [];
for (let i = 0; i < 32; i++) rows.push(blank());
rows[0] = {chord: (1<<15)|(1<<0), gesture:0, flags:1, hold_cs:75,
           keys:[0xE0,0x0D,0,0], rel_order:1, label:'he said "hi" & <b>', present:true};
rows[1] = {chord:0, gesture:0x80|0x03, flags:0, hold_cs:0,
           keys:[0xE3,0x0A,0,0], rel_order:1, label:"O'Brien", present:true};
M.setRows(rows);

M.macroRender();
scanHandlers(boxHtml, 'rows');
ok(boxHtml.indexOf('he said "hi"') < 0, 'a quote in a macro name is escaped, not emitted raw');
ok(/Ctrl \+ J/.test(boxHtml), 'combo renders from keys[]');
ok(/hold 0\.75s/.test(boxHtml), 'long-press threshold shown');

// Picker open: the panel with its checkboxes and the whole key list.
M.setPick({ row: 1, mods: [0xE3], key: 0x0A });
M.macroRender();
scanHandlers(boxHtml, 'picker');
ok(/macroPickApply\(\)/.test(boxHtml), 'picker renders a Set action');
ok((boxHtml.match(/<option /g) || []).length > 60, 'key list is populated');

// The picker must encode combos the RECORDER cannot capture at all.
function pickEncode(mods, key){
  M.setPick({ row: 0, mods: mods.slice(), key });
  M.macroPickApply();
  return M.getRows()[0];
}
let r = pickEncode([0xE3], 0x0A);                      // Win+G
ok(r.keys[0] === 0xE3 && r.keys[1] === 0x0A, 'Win+G stored in press order');
ok(r.rel_order === 1, 'Win+G releases G before Win');
r = pickEncode([0xE2], 0x2B);                          // Alt+Tab
ok(r.keys[0] === 0xE2 && r.keys[1] === 0x2B && r.rel_order === 1, 'Alt+Tab encoded correctly');
r = pickEncode([0xE0, 0xE1], 0x0D);                    // Ctrl+Shift+J
ok(r.keys[2] === 0x0D && r.rel_order === ((2)|(1<<2)|(0<<4)), 'three-key combo reverses on release');

// Every stored rel_order must be a real permutation, or macro.cpp strands a key.
function isPerm(rel, n){
  const seen = new Set();
  for (let i = 0; i < n; i++){ const p = (rel >> (2*i)) & 3; if (p >= n || seen.has(p)) return false; seen.add(p); }
  return true;
}
let allPerm = true;
for (const mods of [[],[0xE0],[0xE0,0xE1],[0xE0,0xE1,0xE2]]){
  const e = pickEncode(mods, 0x04);
  if (!isPerm(e.rel_order, mods.length + 1)) allPerm = false;
}
ok(allPerm, 'picker always emits a valid release permutation');

// The dirty banner is drawn from savedSnapshot. saveAll()'s common path refreshes
// the snapshot but deliberately does NOT render, so the panel must repaint itself
// after a save or it keeps claiming unsaved changes.
(function bannerClears(){
  console.log('dirty banner tracks the saved snapshot');
  const rows2 = []; for (let i=0;i<32;i++) rows2.push(blank());
  rows2[0] = {chord:1<<15, gesture:0, flags:0, hold_cs:0, keys:[0xE0,0x0D,0,0],
              rel_order:1, label:'t', present:true};
  M.setRows(rows2);
  M.setPick(null);
  M.setCfg({macro_disable: 0xFFFFFFFF});
  M.setSnap({macro_disable: 0xFFFFFFFF});
  M.macroRender();
  ok(!/Enable state changed/.test(boxHtml), 'hidden when config matches the snapshot');
  M.macroSetBit(0, true);
  ok(/Enable state changed/.test(boxHtml), 'shown after a checkbox change');
  M.setSnap({...M.getCfg()});          // what a successful saveAll() does
  M.macroRender();                     // the repaint macroSaveTable performs
  ok(!/Enable state changed/.test(boxHtml), 'cleared once the snapshot is refreshed');
})();

// The macro file format must carry the motion fields. Dropping them does not
// just lose the gesture: macro_is_motion() requires GEST_MOTION and
// motion_len > 0, so a stripped motion macro reloads as a plain CHORD macro and
// its gate button fires it on its own, with nothing drawn.
(function motionSurvivesRoundTrip(){
  console.log('macro file format carries motion gestures');
  const rows3 = []; for (let i=0;i<32;i++) rows3.push(blank());
  rows3[2] = {chord:1<<10, gesture:0x40, flags:0, hold_cs:0, keys:[0xE0,0x0D,0,0],
              rel_order:1, label:'down-up', motion:[0x0D,0x00], motion_len:2,
              motion_step:2100, present:true};
  const j = M.macroTableToJson(rows3);
  const back = M.macroTableFromJson(j)[2];
  ok(JSON.stringify(back.motion) === JSON.stringify([0x0D,0x00]), 'motion strokes survive');
  ok(back.motion_len === 2, 'motion_len survives');
  ok(back.motion_step === 2100, 'the calibrated step survives');
  ok(back.gesture === 0x40, 'GEST_MOTION survives - it does not degrade to a chord macro');

  const legacy = M.macroTableFromJson(
    [{index:5,label:'legacy',chord:1<<15,gesture:0,flags:0,hold_cs:0,keys:[0xE0,0x0D,0,0],rel_order:1}])[5];
  ok(legacy.motion_len === 0, 'a pre-1.20.0 file imports as a non-motion macro');

  const bad = M.macroTableFromJson([{index:0,motion_len:200,chord:1,keys:[4,0,0,0]}])[0];
  ok(bad.motion_len <= 8, 'an out-of-range motion_len is clamped on import');
})();

console.log('\nmacro panel handlers checked:', checked, '| problems:', bad);
console.log(bad ? 'MACRO PANEL TEST FAILED' : 'MACRO PANEL TEST OK');
process.exit(bad ? 1 : 0);
