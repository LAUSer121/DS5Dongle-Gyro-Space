// Flick Stick, tested against Jibb Smart's specification
// (gyrowiki.jibbsmart.com "Good Gyro Controls Part 2"). The maths here mirrors
// flick_stick_step() in src/main.cpp.
#include <cstdio>
#include <cmath>
#include <cstdint>

constexpr float FLICK_THRESHOLD = 0.9f, FLICK_TIME_S = 0.10f;
constexpr float TURN_SMOOTH_THRESHOLD = 0.1f;
constexpr int   N = 8;

static float lx=0, ly=0, prog=FLICK_TIME_S, size=0, buf[N]; static int bi=0;
static void zero(){ for(int i=0;i<N;i++) buf[i]=0; }
static void reset(){ lx=ly=0; prog=FLICK_TIME_S; size=0; bi=0; zero(); }
static float warp(float t){ float f=1.0f-t; return 1.0f-f*f; }
static float wrap(float a){ while(a>3.14159265f)a-=6.28318531f; while(a<-3.14159265f)a+=6.28318531f; return a; }
static float smoothed(float in){ bi=(bi+1)%N; buf[bi]=in; float s=0; for(int i=0;i<N;i++)s+=buf[i]; return s/N; }
static float tiered(float in){ float t1=TURN_SMOOTH_THRESHOLD*0.5f,t2=TURN_SMOOTH_THRESHOLD;
  float w=(fabsf(in)-t1)/(t2-t1); if(w<0)w=0; if(w>1)w=1; return in*w + smoothed(in*(1.0f-w)); }
static float step(float sx,float sy,float dt){
  float r=0, len=sqrtf(sx*sx+sy*sy), ll=sqrtf(lx*lx+ly*ly);
  if(len>=FLICK_THRESHOLD){ if(ll<FLICK_THRESHOLD){ prog=0; size=atan2f(-sx,sy); }
    else { float a=atan2f(-sx,sy), la=atan2f(-lx,ly); r+=tiered(wrap(a-la)); } }
  else if(ll>=FLICK_THRESHOLD) zero();
  if(prog<FLICK_TIME_S){ float last=prog; prog+=dt; if(prog>FLICK_TIME_S)prog=FLICK_TIME_S;
    r += (warp(prog/FLICK_TIME_S)-warp(last/FLICK_TIME_S))*size; }
  lx=sx; ly=sy; return r;
}

static int fails=0;
static void ok(bool c,const char*m){ printf(c?"  ok    %s\n":"  FAIL  %s\n",m); if(!c)fails++; }
static float deg(float r){ return r*180.0f/3.14159265f; }

int main(){
  printf("=== flick stick (Jibb Smart spec) ===\n");

  // A flick to the right: stick full right => +90 degrees, delivered over FlickTime.
  reset();
  float total=0;
  total += step(1.0f, 0.0f, 0.001f);
  for(int i=0;i<200;i++) total += step(1.0f, 0.0f, 0.001f);   // 0.2s, past FlickTime
  ok(fabsf(deg(total) + 90.0f) < 0.5f, "full-right flick is -90 deg (spec: anticlockwise positive)");
  ok(deg(total) < 0.0f, "and a RIGHT flick is negative, so the mouse conversion must negate");

  // A 180: stick full back.
  reset(); total=0;
  for(int i=0;i<201;i++) total += step(0.0f, -1.0f, 0.001f);
  ok(fabsf(fabsf(deg(total)) - 180.0f) < 0.5f, "stick-back flick turns 180 degrees");

  // Below the threshold nothing happens at all - the deadzone is deliberate.
  reset(); total=0;
  for(int i=0;i<200;i++) total += step(0.85f, 0.0f, 0.001f);
  ok(total == 0.0f, "stick at 0.85 (below 0.9 threshold) does nothing");

  // The flick completes within FlickTime and then stops.
  reset();
  float during=0, after=0;
  for(int i=0;i<100;i++) during += step(1.0f,0.0f,0.001f);   // exactly 0.1s
  for(int i=0;i<100;i++) after  += step(1.0f,0.0f,0.001f);
  ok(fabsf(fabsf(deg(during))-90.0f) < 0.5f, "the whole flick lands inside FlickTime");
  ok(fabsf(after) < 1e-4f, "and nothing more is delivered afterwards");

  // Ease-out: more than half the movement in the first half of the window.
  reset();
  float firstHalf=0;
  for(int i=0;i<50;i++) firstHalf += step(1.0f,0.0f,0.001f);
  ok(fabsf(deg(firstHalf)) > 45.0f, "ease-out front-loads the movement (no ease-in)");

  // Turning: rotating a held stick adds the angle change. Start the sweep from
  // where the stick already is - jumping there first would itself be a turn.
  // Steps of 6 deg (0.105 rad) sit ABOVE TurnSmoothThreshold, so they pass
  // through directly and the total must match exactly.
  reset();
  for(int i=0;i<200;i++) step(0.0f, 1.0f, 0.001f);            // settle pointing forward
  float turn=0;
  for(int i=1;i<=15;i++){                                      // 15 x 6 = 90 deg
    float a = -i*6.0f*3.14159265f/180.0f;                      // sweep toward the right
    turn += step(-sinf(a), cosf(a), 0.001f);
  }
  printf("       (swept 90 deg in 6-deg steps, camera turned %.1f deg)\n", fabsf(deg(turn)));
  ok(fabsf(fabsf(deg(turn))-90.0f) < 0.5f, "rotating a held stick turns by the same angle");

  // Small movements DO get smoothed - that is the point of soft tiered
  // smoothing - so they lag slightly, but the displacement is nearly honoured.
  reset();
  for(int i=0;i<200;i++) step(0.0f, 1.0f, 0.001f);
  float fine=0;
  for(int i=1;i<=90;i++){                                      // 90 x 1 deg, sub-threshold
    float a = -i*1.0f*3.14159265f/180.0f;
    fine += step(-sinf(a), cosf(a), 0.001f);
  }
  printf("       (90 x 1-deg steps, smoothed: %.1f deg)\n", fabsf(deg(fine)));
  ok(fabsf(deg(fine)) > 80.0f && fabsf(deg(fine)) <= 90.5f,
     "sub-threshold movement is smoothed but still honours the displacement");

  // Releasing mid-turn must not leave a smoothed tail.
  reset();
  for(int i=0;i<200;i++) step(1.0f,0.0f,0.001f);
  step(0.0f,0.0f,0.001f);
  float tail=0;
  for(int i=0;i<20;i++) tail += step(0.0f,0.0f,0.001f);
  ok(tail == 0.0f, "releasing the stick stops immediately, no smoothed tail");

  // Wrap: crossing the back must take the short way round.
  reset();
  for(int i=0;i<200;i++) step(-0.1f,-0.99f,0.001f);          // settle near 180
  float w=0;
  w += step(0.1f,-0.99f,0.001f);
  ok(fabsf(deg(w)) < 30.0f, "crossing the rear seam takes the short path, not 350 degrees");

  // --- radians -> mouse counts -----------------------------------------------
  // A flick is delivered over ~100 ticks. Truncating each one toward zero loses
  // about half a count per tick and ALWAYS in the same direction, so the error
  // compounds: ~2.8 deg per 180 at 6500 counts/360, ~11 deg after four flicks.
  // That is the drift-after-a-few-flicks report. The remainder must be carried.
  {
    const float TWO_PI = 6.28318531f;
    const int   C360   = 6500;
    float rem = 0.0f;
    int32_t sent = 0;
    reset();
    // one full 180 flick, tick by tick, exactly as gyro_mouse_task does it
    for (int i = 0; i < 200; i++) {
      const float yaw = step(0.0f, -1.0f, 0.001f);
      if (yaw == 0.0f) continue;
      const float counts = -(yaw / TWO_PI) * (float) C360 + rem;
      const int32_t whole = (int32_t) counts;
      rem = counts - (float) whole;
      sent += whole;
    }
    const int32_t ideal = C360 / 2;
    printf("       (180 flick: sent %d counts, ideal %d)\n", sent < 0 ? -sent : sent, ideal);
    ok((sent < 0 ? -sent : sent) == ideal, "a 180 flick delivers the exact count, no truncation loss");

    // Ten flicks must not accumulate error - this is the reported symptom.
    reset(); rem = 0.0f; sent = 0;
    for (int f = 0; f < 10; f++) {
      for (int i = 0; i < 150; i++) {
        const float yaw = step(0.0f, -1.0f, 0.001f);
        if (yaw != 0.0f) {
          const float c = -(yaw / TWO_PI) * (float) C360 + rem;
          const int32_t w = (int32_t) c; rem = c - (float) w; sent += w;
        }
      }
      for (int i = 0; i < 30; i++) step(0.0f, 0.0f, 0.001f);   // release between flicks
    }
    const int32_t want = ideal * 10;
    const int32_t got  = sent < 0 ? -sent : sent;
    printf("       (10 flicks: %d counts, ideal %d, error %.2f deg)\n",
           got, want, (float)(want-got) / (float) C360 * 360.0f);
    ok(got == want, "ten flicks in a row accumulate no drift");
  }

  printf(fails ? "\nFLICK TESTS FAILED (%d)\n" : "\nFLICK TESTS OK (0 failures)\n", fails);
  return fails?1:0;
}
