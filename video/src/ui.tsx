import React, { useState } from 'react';
import { AbsoluteFill, OffthreadVideo, Freeze, staticFile, continueRender, delayRender, useCurrentFrame } from 'remotion';
import { C, D, FONT } from './theme';
import { useT } from './i18n';

// ---- fonts (KoPub Dotum from public/) ----
let fontsLoaded = false;
export const useFonts = () => {
  const [handle] = useState(() => (fontsLoaded ? null : delayRender('fonts')));
  if (handle !== null && !fontsLoaded) {
    fontsLoaded = true;
    const faces = [
      new FontFace('KoPub Dotum', `url(${staticFile('KoPubDotumLight.ttf')})`, { weight: '300' }),
      new FontFace('KoPub Dotum', `url(${staticFile('KoPubDotumMedium.ttf')})`, { weight: '500' }),
      new FontFace('KoPub Dotum', `url(${staticFile('KoPubDotumBold.ttf')})`, { weight: '700' }),
    ];
    Promise.all(faces.map((f) => f.load())).then((ls) => {
      ls.forEach((f) => (document as any).fonts.add(f));
      continueRender(handle);
    });
  }
};

export const Page: React.FC<{ children: React.ReactNode }> = ({ children }) => (
  <AbsoluteFill style={{ background: C.bg, color: C.text, fontFamily: FONT, fontWeight: 500, wordBreak: 'keep-all' }}>{children}</AbsoluteFill>
);

// ---- clips ----
export const clipInfo = (name: string) => D.clips?.[name] as { frames: number; start: number; events: any[]; phases?: { f: number; phase: string; item?: number }[] } | undefined;
// current robot motion phase (from the sim's phases.json) at an absolute output frame
export const phaseAt = (name: string, sf: number) => {
  const ph = clipInfo(name)?.phases ?? [];
  let cur: { f: number; phase: string; item?: number } | undefined;
  for (const p of ph) if (p.f <= sf) cur = p;
  return cur;
};

const Placeholder: React.FC<{ name: string; style?: React.CSSProperties }> = ({ name, style }) => {
  const t = useT();
  return <div style={{ ...style, background: C.panel, display: 'flex', alignItems: 'center', justifyContent: 'center', color: C.dim, fontSize: 28 }}>{t.waiting}: {name}</div>;
};

// sim clips are wide shots: zoom in on the wall, arm and conveyor so the content reads on a phone
export const ZOOM = 1.22;
const ZOOM_ORIGIN = '56% 52%';
export const Clip: React.FC<{ name: string; rate: number; style?: React.CSSProperties; zoom?: number }> = ({ name, rate, style, zoom = ZOOM }) => {
  const f = useCurrentFrame();
  const info = clipInfo(name);
  if (!info) return <Placeholder name={name} style={style} />;
  const last = Math.max(0, Math.floor((info.frames - 2) / rate));
  const v = <OffthreadVideo src={staticFile(`clips/${name}.mp4`)} playbackRate={rate} muted style={{ width: '100%', height: '100%', objectFit: 'cover', transform: `scale(${zoom})`, transformOrigin: ZOOM_ORIGIN }} />;
  return <div style={{ ...style, overflow: 'hidden', background: '#000' }}>{f > last ? <Freeze frame={last}>{v}</Freeze> : v}</div>;
};

// a single frame of a clip (clip-local index from 0)
export const ClipAt: React.FC<{ name: string; idx: number; style?: React.CSSProperties; zoom?: number }> = ({ name, idx, style, zoom = ZOOM }) => {
  const info = clipInfo(name);
  if (!info) return <Placeholder name={name} style={style} />;
  return (
    <div style={{ ...style, overflow: 'hidden', background: '#000' }}>
      <Freeze frame={Math.max(0, Math.min(info.frames - 1, idx))}>
        <OffthreadVideo src={staticFile(`clips/${name}.mp4`)} muted style={{ width: '100%', height: '100%', objectFit: 'cover', transform: `scale(${zoom})`, transformOrigin: ZOOM_ORIGIN }} />
      </Freeze>
    </div>
  );
};

// timeline: play motion at `rate`, hold on hand-offs so each decision is readable
export type Seg = { c0: number; c1: number; len: number; off: number; ev: any };
export const buildTimeline = (name: string, rate: number, hold = 66) => {
  const info = clipInfo(name);
  if (!info || !info.events?.length) return null;
  const end = info.start + info.frames - 1;
  const raw: Omit<Seg, 'off' | 'len'>[] = [];
  let cur = info.start;
  for (const e of info.events) {
    const f0 = Math.max(cur, Math.min(e.f0, end));
    const f1 = Math.min(e.f1, end);
    if (f0 > cur) raw.push({ c0: cur, c1: f0, ev: null });
    raw.push({ c0: f0, c1: Math.max(f0, f1), ev: e });
    cur = Math.max(cur, f1);
  }
  if (cur < end) raw.push({ c0: cur, c1: end, ev: null });
  let off = 0;
  const segs: Seg[] = raw.map((s) => {
    const len = s.ev?.action === 'handoff' ? hold : s.c1 - s.c0 < 6 ? (s.ev ? hold : 0) : Math.ceil((s.c1 - s.c0) / rate);
    const r = { ...s, len, off };
    off += len;
    return r;
  }).filter((s) => s.len > 0);
  return { segs, total: off, start: info.start };
};
export const atTimeline = (tl: { segs: Seg[]; start: number }, f: number) => {
  const s = tl.segs.find((x) => f < x.off + x.len) ?? tl.segs[tl.segs.length - 1];
  const u = Math.max(0, Math.min(1, (f - s.off) / Math.max(1, s.len)));
  const cf = Math.round(s.c0 + (s.c1 - s.c0) * u);
  const evs = tl.segs.filter((x) => x.ev && x.off <= f).map((x) => x.ev);
  return { idx: cf - tl.start, ev: s.ev ?? evs[evs.length - 1], u: s.ev ? u : 1, done: evs.filter((e) => e !== s.ev || u >= 1) };
};

export const simFrame = (name: string, f: number, rate: number) => {
  const info = clipInfo(name);
  if (!info) return 0;
  return info.start + Math.min(info.frames - 1, Math.floor(f * rate));
};
export const eventsAt = (name: string, sf: number) => {
  const ev = clipInfo(name)?.events ?? [];
  const done = ev.filter((e: any) => e.f1 <= sf);
  const cur = ev.find((e: any) => e.f0 <= sf && sf < e.f1);
  return { done, last: done[done.length - 1], cur };
};

// ---- tool-call chips ----
export type Call = { tool: string; item?: number; face?: string; ok?: any };
const chipColor = (c: Call) => {
  if (c.tool === 'check_grasp') return c.ok === false ? C.red : C.green;
  if (c.tool === 'preview_grasp') return C.blue;
  if (c.tool === 'hand_off') return C.amber;
  if (c.tool === 'execute_grasp') return c.ok === false ? C.red : C.green;
  return C.dim;
};
export const Chip: React.FC<{ c: Call; strong?: boolean }> = ({ c, strong }) => {
  const t = useT();
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8, padding: '5px 11px', border: `1px solid ${strong ? chipColor(c) : C.line}`,
      background: strong ? chipColor(c) : C.bg, fontSize: strong ? 26 : 23, fontWeight: strong ? 700 : 500, whiteSpace: 'nowrap', color: strong ? '#FFFFFF' : C.text }}>
      <span style={{ width: 10, height: 10, background: strong ? '#FFFFFF' : chipColor(c), display: 'inline-block' }} />
      {t.tool[c.tool] ?? c.tool}
      {c.item != null ? ` ${c.item}` : ''}
      {c.face ? ` ${t.face[c.face] ?? c.face}` : ''}
      {c.tool === 'check_grasp' ? ` ${c.ok === false ? t.block : t.pass}` : ''}
    </div>
  );
};

export const stepOf = (c?: Call) =>
  !c ? -1 : c.tool === 'get_scene' ? 0 : c.tool === 'check_grasp' ? 1 : c.tool === 'preview_grasp' ? 2 : 4;

// calls revealed over the first 45% of the decision, then the decision itself
export const revealCalls = (calls: Call[], u: number) => {
  const v = Math.max(0, Math.min(1, u / 0.45));
  const n = Math.max(1, Math.ceil(v * calls.length));
  return { shown: calls.slice(0, n), active: v >= 1 ? 4 : n === calls.length ? 3 : stepOf(calls[n - 1]), decided: v >= 1 };
};

// ---- the LLM band: steps, then Input | LLM | Output ----
export const BAND_H = 290;
export const LlmBand: React.FC<{
  model: string; used: boolean[]; active: number; calls: Call[]; rec?: any; decided?: boolean; rule?: boolean; tag?: string; dimTo?: number;
}> = ({ model, used, active, calls, rec, decided, rule, tag, dimTo = 1 }) => {
  const t = useT();
  let why: string = (rec?.why ?? '').replace(/^This turn's action is set: /, '');
  const wi = why.indexOf('Why ');
  if (wi > 0) why = why.slice(wi);
  const hdr: React.CSSProperties = { fontSize: 20, color: C.sub, letterSpacing: 0.3 };
  return (
    <div style={{ position: 'absolute', left: 0, top: 0, width: '100%', height: BAND_H, borderBottom: `1px solid ${C.line}`, boxSizing: 'border-box', padding: '14px 48px 0', opacity: dimTo }}>
      <div style={{ display: 'flex', alignItems: 'flex-start' }}>
        <div style={{ width: 300, fontSize: 22, color: C.sub, paddingTop: 8 }}>{tag}</div>
        {t.steps.map((s, i) => (
          <div key={s} style={{ width: 272, marginRight: 12, borderTop: `4px solid ${!used[i] ? C.line : i === active ? C.blue : '#C9CACC'}`, padding: '4px 10px 5px',
            background: i === active ? C.blue : 'transparent', opacity: used[i] ? 1 : 0.4, whiteSpace: 'nowrap', overflow: 'hidden' }}>
            <span style={{ fontSize: 28, fontWeight: 700, color: i === active ? '#FFFFFF' : used[i] ? C.sub : C.dim }}>{s}</span>
            <span style={{ fontSize: 19, color: i === active ? '#FFFFFF' : C.dim, marginLeft: 8 }}>{used[i] ? t.stepSub[i] : t.unused}</span>
          </div>
        ))}
      </div>
      <div style={{ display: 'flex', marginTop: 14, height: 206 }}>
        <div style={{ width: 300, paddingRight: 24, borderRight: `1px solid ${C.line}` }}>
          <div style={hdr}>{t.input}</div>
          <div style={{ fontSize: 24, color: C.text, marginTop: 8, lineHeight: 1.4 }}>{t.inputTxt}</div>
        </div>
        <div style={{ width: 400, padding: '0 24px', borderRight: `1px solid ${C.line}` }}>
          <div style={hdr}>{t.model}</div>
          <div style={{ fontSize: 32, fontWeight: 700, marginTop: 6, lineHeight: 1.15 }}>{rule ? '—' : model}</div>
          <div style={{ fontSize: 22, color: C.sub, marginTop: 10, lineHeight: 1.4 }}>
            {rule ? t.noLlm : rec ? t.perDecision(rec.lat ?? 0, rec.tok_in, rec.tok_cached, rec.tok_out, rec.cost) : ''}
          </div>
        </div>
        <div style={{ flex: 1, paddingLeft: 24, overflow: 'hidden' }}>
          <div style={hdr}>{t.output}</div>
          <div style={{ display: 'flex', gap: 8, marginTop: 8, overflow: 'hidden' }}>
            {calls.slice(-3).map((c, i, a) => <Chip key={i} c={c} strong={i === a.length - 1} />)}
          </div>
          {!rule && why && (
            <div style={{ marginTop: 12, opacity: decided ? 1 : 0.5 }}>
              <div style={hdr}>{t.why}</div>
              <div style={{ fontSize: 25, lineHeight: 1.35, marginTop: 4, color: C.text, display: '-webkit-box', WebkitLineClamp: 2, WebkitBoxOrient: 'vertical' as any, overflow: 'hidden' }}>
                “{why}”
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
