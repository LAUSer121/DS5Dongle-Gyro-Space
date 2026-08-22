const fs=require('fs'); let js=fs.readFileSync('/tmp/portal.js','utf8');
const stop=js.indexOf('\nrender();\nportalSelfCheck();'); js=js.slice(0,stop>0?stop:js.length);
global.setInterval=()=>0;global.setTimeout=()=>0;global.requestAnimationFrame=()=>0;
global.window={setInterval:()=>0,setTimeout:()=>0};
global.document={getElementById:()=>null,createElement:()=>({style:{},classList:{add(){}}}),body:{prepend(){}},addEventListener(){},querySelector:()=>null};
global.navigator={hid:{getDevices:async()=>[],addEventListener(){}}};
// Extract the LIVE motion functions from the portal, never a copy: a baked-in
// copy passes forever while the portal drifts underneath it.
const M=new Function('window','document','navigator','location',
  js+` return { motNewState, motFeed, motMatch, motDir, motPathName, motSuggestStep, motQuantise, motFind };`)
 (global.window,global.document,global.navigator,global.location);
const {motNewState, motFeed, motMatch, motDir, motPathName, motSuggestStep, motQuantise, motFind} = M;

function fakeReport(yaw, pitch){
  const b = new Uint8Array(64); const dv = new DataView(b.buffer);
  dv.setInt16(17, yaw, true); dv.setInt16(15, pitch, true);
  return dv;
}
function trace(legs, step){
  const ms = motNewState();
  for (const [y,p,n] of legs) for (let k=0;k<n;k++) motFeed(ms, fakeReport(y,p));
  global.window._motStep = step || 1800;
  return motQuantise(ms.raw, step || 1800).codes;
}
let pass=0, fail=0;
function ck(name, got, want){
  const ok = JSON.stringify(got)===JSON.stringify(want);
  console.log((ok?'  ok   ':'  FAIL ')+name+'   got '+motPathName(got)+' ['+got+']');
  ok?pass++:fail++;
}
global.window._motStep = 1800;
ck('flick right',  trace([[-500,0,10]]), [0]);
ck('flick left',   trace([[ 500,0,10]]), [2]);
ck('flick up',     trace([[0,-500,10]]), [1]);
ck('flick down',   trace([[0, 500,10]]), [3]);
ck('right then up (L shape)', trace([[-500,0,10],[0,-500,10]]), [0,1]);
ck('long sweep is ONE stroke', trace([[-500,0,60]]), [0]);
ck('below threshold emits nothing', trace([[-50,0,10]]), []);
ck('zigzag R U R', trace([[-500,0,10],[0,-500,10],[-500,0,10]]), [0,1,0]);
ck('U shape down-right-up', trace([[0,500,10],[-500,0,10],[0,-500,10]]), [3,0,1]);
ck('resting bias never emits', trace([[8,4,400]]), []);
ck('drift under noise floor ignored', trace([[15,0,300]]), []);
// THE REGRESSION THAT KILLED THE 8-WAY VERSION. A wobbly up-flick: mostly up,
// drifting left, with the tremor that a real hand has. It produced
// "up up-left up up-left up ..." to the 12-code ceiling. It must be ONE stroke.
const wobble = [];
for (let k=0;k<40;k++) wobble.push([k%2 ? -90 : 60, -500, 1]);
ck('wobbly up-flick is one stroke', trace(wobble), [1]);
// A flick that overshoots and springs back should not add a phantom return
// stroke unless the return is genuinely large.
ck('small overshoot ignored', trace([[0,-500,10],[0,200,2]]), [1]);
ck('real reversal is a second stroke', trace([[0,-500,10],[0,500,10]]), [1,3]);
const mm=(a,b)=>motMatch(a,b);
function ck2(n,g,w){const ok=g===w;console.log((ok?'  ok   ':'  FAIL ')+n);ok?pass++:fail++;}
ck2('exact match', mm([0,1],[0,1]), true);
ck2('different path rejected', mm([0,1],[0,3]), false);
ck2('leading jitter tolerated', mm([3,0,1],[0,1]), true);
ck2('trailing jitter tolerated', mm([0,1,3],[0,1]), true);
// Two spurious codes BEFORE the gesture: under prefix matching only one may be
// dropped, so the opening must be nearly right. (The old form of this case,
// [3,0,1,3], is now correctly a MATCH - one leading extra plus trailing
// settling - so it was testing the absence of a feature that now exists.)
ck2('two extra leading codes rejected', mm([3,2,0,1],[0,1]), false);
ck2('empty capture never matches', mm([],[0,1]), false);
ck2('reversed path rejected', mm([1,0],[0,1]), false);
// Calibration suggestion should land a typical gesture at ~3 codes.
const ms = motNewState();
for (const [y,p,n] of [[-500,0,10],[0,-500,10]]) for (let k=0;k<n;k++) motFeed(ms, fakeReport(y,p));
const sug = motSuggestStep(ms);
ck2('suggested step is a sane positive number', sug >= 200 && sug < 200000, true);

// --- diagonal and dither cases, from real captures that broke earlier builds --
// A 45-degree up-left drag. It sits exactly on the boundary between two
// 90-degree sectors; the un-hystereticised version read it as
// "up left up left up left down left". It must be ONE stroke.
ck('45-degree diagonal is one stroke', trace([[400,-400,30]]).length <= 1, true);
// Same shape, the other diagonal.
ck('down-right diagonal is one stroke', trace([[-400,400,30]]).length <= 1, true);
// A near-diagonal that clearly favours one axis resolves to that axis.
ck('mostly-up diagonal reads up', trace([[150,-500,20]]), [1]);
// A genuine right-angle turn must still register.
ck('right angle still turns', trace([[-500,0,12],[0,-500,12]]), [0,1]);

// --- reversal cases, from the 4-of-6 hardware run -----------------------------
// A down-then-up where the RETURN stroke is weaker than the outbound one. At the
// old 2x reversal threshold this was swallowed and captured as just "down".
ck('weak return stroke still reverses', trace([[0,500,12],[0,-300,12]]), [3,1]);
// A down-up performed with lateral drift. The drift used to cross the turn
// threshold before the reversal did, injecting a phantom "right" mid-path.
ck('reversal beats lateral drift', trace([[-120,500,12],[-120,-500,12]]), [3,1]);
// Still must not invent a reversal out of spring-back.
ck('spring-back is not a reversal', trace([[0,-600,14],[0,120,3]]), [1]);
const mm2=(a,b)=>motMatch(a,b);
ck2('spurious code in the MIDDLE tolerated', mm2([3,0,1],[3,1]), true);
ck2('spurious code at the front tolerated', mm2([1,3,1],[3,1]), true);
ck2('two spurious codes still rejected', mm2([1,3,0,1],[3,1]), false);
ck2('a genuinely different 2-code path is rejected', mm2([0,1],[3,1]), false);

// --- long-hold cases, from the 21-of-22 hardware run --------------------------
// The gate window ends at RELEASE, so holding the button after finishing the
// gesture sweeps up settling motion. This exact capture failed before prefix
// matching: a clean down-up followed by hand settling.
ck2('trailing settling motion ignored', mm2([3,1,3,1,0,3],[3,1]), true);
ck2('gesture at the front still matches with drift', mm2([3,0,1,3,1],[3,1]), true);
// The gesture must still be at the START. A window that begins with something
// else and only later contains the template is NOT a match - otherwise a long
// hold during normal play eventually contains any short template by chance.
ck2('template later in the window is not a match', mm2([0,0,3,1],[3,1]), false);
ck2('single wrong opening stroke still rejected', mm2([2,0,0],[3,1]), false);
// Longest template wins so a short one does not shadow a longer one.
global.window._macroMotion = [];
global.window._macroMotion[1] = {codes:[3,1], gate:0};
global.window._macroMotion[2] = {codes:[3,1,0], gate:0};
ck2('longer template preferred over its own prefix', M.motFind([3,1,0], 0), 2);
ck2('short template still wins when the long one does not apply', M.motFind([3,1,2], 0), 1);
global.window._macroMotion = [];

// --- single-stroke templates, from the 33-window hardware run ------------------
// Four single-stroke templates on one gate. Every one of these produced an
// AMBIGUOUS match before the slack was made proportional to template length.
ck2('left-flick drifting down matches only left', mm2([2,3],[2]), true);
ck2('left-flick drifting down does NOT match down', mm2([2,3],[3]), false);
ck2('up-then-left does NOT match left', mm2([1,2],[2]), false);
ck2('down-then-up does NOT match up', mm2([3,1],[1]), false);
ck2('single-stroke template still matches itself', mm2([3],[3]), true);
ck2('single-stroke still matches with trailing settling', mm2([3,1,0],[3]), true);
// The slack must SURVIVE for multi-stroke templates - that is what it is for.
ck2('two-stroke template keeps its drift tolerance', mm2([3,0,1],[3,1]), true);
// Four orthogonal single-stroke templates must be mutually exclusive.
global.window._macroMotion = [];
[[1,[2]],[2,[0]],[3,[3]],[4,[1]]].forEach(([r,c]) => { global.window._macroMotion[r] = {codes:c, gate:0}; });
[[[2,3],1],[[0,1],2],[[3,0],3],[[1,2],4]].forEach(([cap,want]) => {
  const got = M.motFind(cap, 0);
  ck2('orthogonal set: ' + motPathName(cap) + ' -> row ' + want, got === want && !global.window._motAmbig, true);
});
global.window._macroMotion = [];
console.log('\npass '+pass+'  fail '+fail);
process.exit(fail?1:0);
