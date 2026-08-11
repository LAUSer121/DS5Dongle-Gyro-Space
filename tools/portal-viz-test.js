// Effect-map visualizer guard. The picture MUST agree with the reader: the same
// sequencer maths decides where a stage sits and whether its break/snap is
// reached, so a wall that the reader flags as handed-over early must also show a
// faded/✕ break in the SVG, and a clean non-overlapping layout must show none.
// Pure function (no DOM), so this exercises it directly on the real captures.
const fs = require('fs');
let js = fs.readFileSync('/tmp/portal.js', 'utf8');
const stop = js.indexOf('\nrender();\nportalSelfCheck();');
js = js.slice(0, stop > 0 ? stop : js.length);
global.setInterval = () => 0; global.setTimeout = () => 0; global.requestAnimationFrame = () => 0;
global.window = { setInterval: () => 0, setTimeout: () => 0 };
global.document = { getElementById: () => null, createElement: () => ({ style: {}, classList: { add() {} } }),
                    body: { prepend() {} }, addEventListener() {}, querySelector: () => null };
global.navigator = { hid: { getDevices: async () => [], addEventListener() {} } };
const M = new Function('window', 'document', 'navigator', 'location', js + `
  return { ceEffectSVG, ceEffectPanel, cebBuildBytes, ceZonesOfBytes, ceActiveWindows };`)
  (global.window, global.document, global.navigator, global.location);

const hex = s => s.split(' ').map(x => parseInt(x, 16)).concat(Array(11).fill(0)).slice(0, 11);
const crosses = s => (s.match(/✕/g) || []).length;
let fails = 0;
const ok = (cond, msg) => { console.log(`  ${cond ? 'ok  ' : 'FAIL'} ${msg}`); if (!cond) fails++; };

// --- well-formedness --------------------------------------------------------
ok(M.ceEffectSVG([]) === '', 'empty input -> empty string (nothing to draw)');
const single = M.ceEffectSVG([M.cebBuildBytes({ type: 'break', start: 2, end: 5, strength: 100 })]);
ok(single.startsWith('<svg') && single.trimEnd().endsWith('</svg>'), 'single wall -> well-formed <svg>');

// --- a lone wall reaches its own break (no sequencer to cut it) --------------
ok(single.includes('break') && crosses(single) === 0, 'lone wall 2->5: break is reached, no ✕');

// --- the R&C-style set the reader flags as truncated (wall 4->8 + resist 7) --
// reader-test proves: wall handed over at 166, breaks at 204 -> not reached.
const trunc = M.ceEffectSVG([{ bytes: hex('25 10 01 03') }, { bytes: hex('21 80 03') }]);
ok(crosses(trunc) >= 1, 'wall handed over before its break -> ✕ shown (matches reader)');

// --- a clean non-overlapping two-wall layout: both breaks reached ------------
const clean = M.ceEffectSVG([{ bytes: hex('25 14 00 07') }, { bytes: hex('25 a0 00 07') }]); // zones 2,4 / 5,7
const w = M.ceActiveWindows([hex('25 14 00 07'), hex('25 a0 00 07')]);
ok(w && Object.keys(w).length === 2, 'two-wall layout yields two sequencer windows');
ok(crosses(clean) === 0, 'non-overlapping walls: every break reached, no ✕');

// --- a bow draws and snaps ---------------------------------------------------
const bow = M.ceEffectSVG([{ bytes: hex('22 88 00 19') }]); // bow 3->7
ok(bow.includes('snap') && crosses(bow) === 0, 'lone bow 3->7: snap reached, no ✕');

// --- all-vibration timeline routes to the time strip, not a position diagram -
const tl = M.ceEffectSVG([{ bytes: hex('26 00 03 00 00 00 09 00 00 2d'), dt: 120 },
                          { bytes: hex('26 00 03 00 00 00 05 00 00 40'), dt: 80 }]);
ok(tl.includes('time →') && tl.includes('Time-based'), 'all-vibration multi-state -> time strip');

// --- a vibration in a MIXED set is a positioned stage, NOT position-independent.
// Firmware run_sequencer sorts it by start zone and gives it a real window.
const loneVib = [hex('26 00 03 00 00 00 09 00 00 2d')];               // zones 8,9
ok(M.ceActiveWindows(loneVib) === null, 'a lone vibration is not sequenced (plays wherever armed)');
const mixed = [hex('25 14 00 07'), hex('26 00 03 00 00 00 09 00 00 2d')]; // wall 2,4 + vib 8,9
const wm = M.ceActiveWindows(mixed);
ok(wm && wm[1] && wm[1].order === 2 && wm[1].from > 0,
   'vibration in a mixed set gets a positioned window that does not start at 0');
ok(wm && wm[0] && wm[0].order === 1, 'the wall is stage 1 ahead of the vibration');
// and a wall+vibration 2-stage set is now sequenced (old code needed 2 mechanical)
ok(M.ceActiveWindows([hex('25 14 00 07'), hex('26 40 00 07')]) !== null,
   'wall + vibration (one mechanical) is sequenced');

// --- panel wrapper carries the legend + svg ---------------------------------
const panel = M.ceEffectPanel([M.cebBuildBytes({ type: 'break', start: 2, end: 5, strength: 100 })], 'x');
ok(panel.includes('Weapon break') && panel.includes('<svg'), 'panel wrapper includes legend + svg');

console.log(fails ? 'VIZ TEST FAILED' : 'VIZ TEST OK');
if (fails) process.exit(1);
