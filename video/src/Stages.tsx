import React from 'react';
import { useCurrentFrame, interpolate } from 'remotion';
import { C, D, FPS } from './theme';
import { useT } from './i18n';
import { Page, Clip, ClipAt, clipInfo, simFrame, eventsAt, phaseAt, LlmBand, BAND_H, revealCalls, buildTimeline, atTimeline, Call } from './ui';

type Verdict = 'fail' | 'drop' | 'partial' | 'pass';
export type Stage = { n: number; clip: string; used: boolean[]; verdict: Verdict; trace?: string; rule?: boolean };

const evOf = (clip: string, item: number) => clipInfo(clip)?.events?.find((e: any) => e.item === item && e.action === 'execute');
// vid_rule300 was rendered before the events patch; vid.log of that render: pick 29 front fell=[28,31,32,33,37,40,41,43,44]
export const R29: number = evOf('vid_rule300', 29)?.fell_now?.length ?? 9;
export const P29: number | undefined = evOf('vid_astra300', 29)?.fell_now?.length;
const ALL = [true, true, true, true, true];
const NOPREV = [true, true, false, true, true];

export const STAGES: Stage[] = [
  { n: 1, clip: 'vid_rule300', used: [true, true, false, false, true], verdict: 'fail', rule: true },
  { n: 2, clip: 'show_fail_opus300', used: NOPREV, verdict: 'fail', trace: 'opus1' },
  { n: 3, clip: 'show_catch_opus300', used: NOPREV, verdict: 'drop', trace: 'opus1' },
  { n: 4, clip: 'vid_astralift300', used: NOPREV, verdict: 'partial', trace: 'llm' },
  { n: 5, clip: 'vid_astra300', used: ALL, verdict: 'pass', trace: 'prev' },
];
export const VCOL: Record<Verdict, string> = { fail: C.red, drop: C.dim, partial: C.amber, pass: C.green };

export const stageRate = (s: Stage) => Math.max(0.5, (clipInfo(s.clip)?.frames ?? 300) / (11 * FPS));
export const stageDur = (s: Stage) => {
  const tl = buildTimeline(s.clip, stageRate(s));
  return (tl ? tl.total : Math.ceil((clipInfo(s.clip)?.frames ?? 300) / stageRate(s))) + 2.5 * FPS;
};

export const StageScene: React.FC<{ s: Stage }> = ({ s }) => {
  const f = useCurrentFrame();
  const t = useT();
  const L = t.stages[s.n - 1];
  const rate = stageRate(s);
  const info = clipInfo(s.clip);
  const tl = buildTimeline(s.clip, rate);
  let e: any = null;
  let u = 0;
  let counts: any = null;
  let idx = 0;
  if (tl) {
    const a = atTimeline(tl, Math.min(f, tl.total - 1));
    e = a.ev; u = a.u; idx = a.idx; counts = a.done[a.done.length - 1];
  } else {
    const sf = simFrame(s.clip, f, rate);
    const ea = eventsAt(s.clip, sf);
    e = ea.cur ?? ea.last;
    if (!e && s.trace && info) e = { t: (D.trace[s.trace] as any[])[0]?.t, f0: info.start, f1: info.start + info.frames, item: undefined };
    u = e ? Math.max(0, Math.min(1, (sf - e.f0) / Math.max(1, e.f1 - e.f0))) : 0;
  }
  let calls: Call[] = [];
  let active = -1;
  let rec: any = null;
  let decided = false;
  if (s.rule) {
    calls = [{ tool: 'check_grasp', item: 29, face: 'front', ok: true }, { tool: 'execute_grasp', item: 29, face: 'front', ok: true }];
    active = f > 20 ? 4 : 1;
  } else if (e && s.trace) {
    rec = (D.trace[s.trace] as any[]).find((r) => r.t === e.t) ?? (D.trace[s.trace] as any[])[0];
    const r = revealCalls(rec?.calls ?? [], u);
    calls = r.shown; active = r.active; decided = r.decided;
  }
  const dur = stageDur(s);
  const resultAt = dur - 2.5 * FPS;
  const ro = interpolate(f, [resultAt - 10, resultAt + 5], [0, 1], { extrapolateLeft: 'clamp', extrapolateRight: 'clamp' });
  const vc = VCOL[s.verdict];
  const top = BAND_H + 16;
  const vidH = 1080 - top - 96;
  const vidW = Math.round((vidH * 16) / 9);
  const modelName = s.trace === 'opus1' ? 'Claude Opus' : 'GPT-6 Astra';
  return (
    <Page>
      <LlmBand tag={L.band} model={modelName} used={s.used} active={active} calls={calls} rec={rec} decided={decided} rule={s.rule} />
      {tl ? <ClipAt name={s.clip} idx={idx} style={{ position: 'absolute', left: 48, top, width: vidW, height: vidH }} />
        : <Clip name={s.clip} rate={rate} style={{ position: 'absolute', left: 48, top, width: vidW, height: vidH }} />}
      <div style={{ position: 'absolute', left: 48 + vidW - 110, top: top + vidH - 40, width: 96, textAlign: 'right', fontSize: 22, color: '#FFFFFF',
        textShadow: '0 0 6px rgba(0,0,0,0.5)' }}>×{rate.toFixed(1)}</div>
      <div style={{ position: 'absolute', left: 48 + 24, top: top + 24, padding: '10px 22px', background: vc, color: '#FFFFFF', fontSize: 44, fontWeight: 700,
        boxShadow: '0 2px 12px rgba(0,0,0,0.2)', transformOrigin: 'left top',
        transform: `scale(${interpolate(f, [15, 21, 25], [0.8, 1.06, 1], { extrapolateLeft: 'clamp', extrapolateRight: 'clamp' })})`, opacity: interpolate(f, [15, 30], [0, 1], { extrapolateLeft: 'clamp', extrapolateRight: 'clamp' }) }}>{t.stageKey(R29)[s.n - 1]}</div>
      <div style={{ position: 'absolute', left: 48 + vidW + 40, top, width: 1920 - 48 - vidW - 40 - 48 }}>
        <div style={{ fontSize: 24, color: C.sub }}>{t.attempt} {s.n}</div>
        <div style={{ fontSize: 50, fontWeight: 700, marginTop: 2, lineHeight: 1.12 }}>{L.name}</div>
        <div style={{ fontSize: 26, color: C.sub, marginTop: 12, lineHeight: 1.4 }}>{L.what}</div>
        {f < resultAt - 10 && e && e.item != null && (
          <div style={{ marginTop: 26, fontSize: 30, lineHeight: 1.4 }}>
            <div style={{ fontSize: 22, color: C.sub }}>{t.decision} {e.t + 1}</div>
            <div style={{ fontWeight: 700 }}>{e.action === 'handoff' ? t.handoffTxt(e.item) : t.pickTxt(e.item, e.face)}</div>
            {u >= 1 && e.fell_now?.length > 0 && <div style={{ color: C.red, fontWeight: 700 }}>{t.fellTxt(e.fell_now.length)}</div>}
            {counts && <div style={{ fontSize: 23, color: C.sub, marginTop: 6 }}>{t.cum(counts.picked, counts.fell, counts.handed)}</div>}
          </div>
        )}
        {f < resultAt - 10 && <MotionSteps clip={s.clip} abs={tl ? idx + tl.start : simFrame(s.clip, f, rate)} handoff={e?.action === 'handoff'} />}
        <div style={{ marginTop: 30, opacity: ro }}>
          <div style={{ display: 'inline-block', padding: '5px 14px', border: `1.5px solid ${vc}`, color: vc, fontSize: 25, fontWeight: 700 }}>{t.verdict[s.verdict]}</div>
          <div style={{ fontSize: 29, marginTop: 14, lineHeight: 1.4 }}>{L.result(R29, P29)}</div>
        </div>
      </div>
      <StageStrip n={s.n} />
    </Page>
  );
};

// live robot-motion steps for the current pick (phases written by the sim), plus the collision rule in one line
export const MotionSteps: React.FC<{ clip: string; abs: number; handoff?: boolean }> = ({ clip, abs, handoff }) => {
  const t = useT();
  const ph = clipInfo(clip)?.phases ?? [];
  if (!ph.length) return null;
  const pull = ph.some((p) => p.phase === 'lift') ? 'lift' : 'pull';
  const rows = ['approach', 'grip', pull, 'transfer', 'place', 'release'];
  const cur = handoff ? undefined : phaseAt(clip, abs);
  const ci = cur ? rows.indexOf(cur.phase) : -1;
  return (
    <div style={{ marginTop: 22 }}>
      <div style={{ fontSize: 22, color: C.sub }}>{t.motionHead}{cur?.item != null ? ` · #${cur.item}` : ''}</div>
      <div style={{ marginTop: 8 }}>
        {rows.map((r, i) => {
          const on = i === ci;
          const done = ci >= 0 && i < ci;
          return (
            <div key={r} style={{ display: 'flex', alignItems: 'center', gap: 12, height: 35, fontSize: 25,
              color: on ? C.text : done ? C.sub : C.dim, fontWeight: on ? 700 : 500 }}>
              <div style={{ width: 13, height: 13, borderRadius: 7, flex: 'none',
                background: on ? C.blue : done ? C.sub : 'transparent', border: `1.5px solid ${on ? C.blue : done ? C.sub : C.line}` }} />
              {t.phase[r]}
            </div>
          );
        })}
      </div>
      <div style={{ fontSize: 22, color: C.sub, marginTop: 8, lineHeight: 1.35 }}>{t.motionRule}</div>
    </div>
  );
};

export const StageStrip: React.FC<{ n: number }> = ({ n }) => {
  const t = useT();
  return (
    <div style={{ position: 'absolute', left: 48, right: 48, bottom: 30, display: 'flex', gap: 8 }}>
      {STAGES.map((s) => {
        const done = s.n < n;
        return (
          <div key={s.n} style={{ flex: 1, borderTop: `3px solid ${s.n === n ? C.text : done ? VCOL[s.verdict] : C.line}`, paddingTop: 7,
            fontSize: 23, color: s.n === n ? C.text : done ? C.sub : C.dim }}>
            {s.n}. {t.stages[s.n - 1].name}{done ? ` · ${t.verdict[s.verdict]}` : ''}
          </div>
        );
      })}
    </div>
  );
};
