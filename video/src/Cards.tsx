import React from 'react';
import { useCurrentFrame, interpolate } from 'remotion';
import { C, D, FPS, pct, mtok, ktok } from './theme';
import { useT } from './i18n';
import { Page, ClipAt, clipInfo } from './ui';
import { R29, P29 } from './Stages';

export const fade = (f: number, d = 12) => interpolate(f, [0, d], [0, 1], { extrapolateRight: 'clamp', extrapolateLeft: 'clamp' });

// ---- hook: same box, two outcomes; title and key numbers come in underneath ----
export const HOOK_DUR = 10 * FPS;
export const Hook: React.FC = () => {
  const f = useCurrentFrame();
  const t = useT();
  const w = 900;
  const h = Math.round((w * 9) / 16);
  const ri = clipInfo('vid_astra300');
  const e16 = ri?.events?.find((e: any) => e.item === 29 && e.action === 'execute');
  const r0 = e16 ? e16.f0 - ri!.start : 0;
  const rEnd = e16 ? e16.f1 - ri!.start : (ri?.frames ?? 1) - 1;
  const li = Math.floor(f * 0.6);
  const rj = Math.min(rEnd, r0 + Math.floor(f * 0.7));
  const show = fade(f - 150, 10);
  const ti = fade(f - 205, 14);
  return (
    <Page>
      <div style={{ position: 'absolute', left: 48, top: 44, fontSize: 56, fontWeight: 700, opacity: fade(f, 8) }}>{t.hookTop}</div>
      {[0, 1].map((k) => (
        <div key={k} style={{ position: 'absolute', left: 48 + k * (w + 24), top: 140, width: w }}>
          {k === 0 ? <ClipAt name="vid_rule300" idx={li} style={{ width: w, height: h }} /> : <ClipAt name="vid_astra300" idx={rj} style={{ width: w, height: h }} />}
          <div style={{ display: 'flex', alignItems: 'baseline', gap: 24, marginTop: 18 }}>
            <div style={{ fontSize: 38, fontWeight: 700, color: C.text }}>{k === 0 ? t.hookL : t.hookR}</div>
            <div style={{ fontSize: 64, fontWeight: 700, color: k === 0 ? C.red : C.green, opacity: show, fontVariantNumeric: 'tabular-nums' }}>
              {t.hookFell(k === 0 ? R29 : P29 ?? 0)}
            </div>
          </div>
        </div>
      ))}
      <div style={{ position: 'absolute', left: 48, right: 48, top: 800, borderTop: `1px solid ${C.line}`, paddingTop: 26, opacity: ti }}>
        <div style={{ fontSize: 24, color: C.dim }}>{t.titleSup}</div>
        <div style={{ fontSize: 46, fontWeight: 700, marginTop: 8 }}>{t.title[0]} {t.title[1]}</div>
        <div style={{ display: 'flex', gap: 56, marginTop: 14, fontSize: 28, fontVariantNumeric: 'tabular-nums', alignItems: 'baseline' }}>
          {t.kpi.map(([k, a, b], i) => (
            <div key={k}>
              <span style={{ color: C.dim, fontSize: 25 }}>{k} </span>
              <span style={{ color: C.dim }}>{a}</span> <span style={{ color: C.dim }}>→</span>{' '}
              <b style={{ color: i === 0 ? C.text : C.green }}>{b}</b>
            </div>
          ))}
          <div style={{ fontSize: 24, color: C.dim }}>{t.kpiNote}</div>
        </div>
      </div>
    </Page>
  );
};

// ---- results table, drawn as a panel over the frozen side-by-side grid ----
export const ResultsPanel: React.FC<{ f: number }> = ({ f }) => {
  const t = useT();
  const w5 = D.stats.w5;
  const w10 = D.stats.w10;
  const o1 = D.stats.opus1;
  const rows = [w5.rule, w5.rulelift, w5.llm, w5.prev];
  const cols = [380, 310, 110, 150, 190, 180, 420];
  const rng = (a: number | null, b: number | null, u: string) => (a == null ? '—' : `${Math.round(a)}–${Math.round(b ?? a)}${u}`);
  return (
    <div style={{ padding: '26px 36px' }}>
      <div style={{ fontSize: 24, color: C.sub }}>{t.resSup}</div>
      <div style={{ fontSize: 42, fontWeight: 700, marginTop: 4 }}>{t.resTitle}</div>
      <div style={{ marginTop: 20 }}>
        <div style={{ display: 'flex', fontSize: 23, color: C.sub, borderBottom: `1px solid ${C.line}`, paddingBottom: 8 }}>
          {t.resHead.map((hd, i) => <div key={hd} style={{ width: cols[i] }}>{hd}</div>)}
        </div>
        {rows.map((r, i) => {
          const llm = i >= 2;
          const p = r.picked / r.n;
          const tok = llm ? `${mtok(r.tok_in)} · ${Math.round((100 * r.tok_cached) / r.tok_in)}% · ${ktok(r.tok_out)}` : '0';
          return (
            <div key={i} style={{ display: 'flex', alignItems: 'center', fontSize: 30, padding: '22px 0', borderBottom: `1px solid ${C.line}`,
              opacity: fade(f - 6 - i * 6), fontVariantNumeric: 'tabular-nums', background: i === 3 ? '#EEF6F1' : 'transparent', outline: i === 3 ? `3px solid ${C.green}` : 'none' }}>
              <div style={{ width: cols[0], fontWeight: 700, color: llm ? C.text : C.sub }}>{t.resRows[i]}</div>
              <div style={{ width: cols[1], display: 'flex', alignItems: 'center', gap: 12 }}>
                <div style={{ width: 140, height: 9, background: C.line }}>
                  <div style={{ width: 140 * p * Math.min(1, fade(f - 10 - i * 6, 20)), height: 9, background: C.green }} />
                </div>
                <span style={i === 3 ? { fontSize: 38, fontWeight: 700, color: C.green } : undefined}>{pct(r.picked, r.n)}</span>
              </div>
              <div style={{ width: cols[2], color: r.fell <= 10 ? C.green : C.red, fontWeight: 700, fontSize: i === 3 ? 40 : undefined }}>{r.fell}</div>
              <div style={{ width: cols[3], color: C.amber }}>{r.handoff}</div>
              <div style={{ width: cols[4] }}>{llm ? rng(r.lat_min, r.lat_max, ' s') : t.instant}</div>
              <div style={{ width: cols[5] }}>{rng(r.run_min, r.run_max, t.min)}</div>
              <div style={{ width: cols[6], color: llm ? C.text : C.dim }}>{tok}</div>
            </div>
          );
        })}
      </div>
      <div style={{ marginTop: 22, fontSize: 24, color: C.sub, lineHeight: 1.6, opacity: fade(f - 40) }}>
        <div>{t.res10(pct(w10.rule.picked, w10.rule.n), w10.rule.fell, pct(w10.llm.picked, w10.llm.n), w10.llm.fell, w10.llm.run_sum_h.toFixed(1), mtok(w10.llm.tok_in))}</div>
        <div>{t.resOpus(o1.picked, o1.n, o1.fell, o1.handoff, Math.round(o1.lat_min), o1.cost.toFixed(2), ktok(o1.tok_out))}</div>
        <div>{t.resAstra(ktok(w5.prev.tok_in / w5.prev.decisions))}</div>
      </div>
    </div>
  );
};

// ---- ending: full-frame sim of the preview agent with the conclusion over it ----
export const END_DUR = 10 * FPS;
export const Ending: React.FC = () => {
  const f = useCurrentFrame();
  const t = useT();
  const w5 = D.stats.w5;
  const info = clipInfo('grid_astraprev300');
  const n = info?.frames ?? 1;
  const idx = Math.max(0, n - END_DUR - 1) + f;
  return (
    <Page>
      <ClipAt name="grid_astraprev300" idx={idx} style={{ position: 'absolute', left: 0, top: 0, width: 1920, height: 1080 }} />
      <div style={{ position: 'absolute', left: 0, top: 0, width: 1060, height: 1080,
        background: 'linear-gradient(90deg, rgba(255,255,255,0.97) 0%, rgba(255,255,255,0.94) 78%, rgba(255,255,255,0) 100%)' }} />
      <div style={{ position: 'absolute', left: 96, top: 150, width: 860 }}>
        <div style={{ fontSize: 28, color: C.sub, opacity: fade(f) }}>{t.conSup}</div>
        <div style={{ fontSize: 58, fontWeight: 700, marginTop: 4, opacity: fade(f) }}>{t.conTitle}</div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 14, marginTop: 22, whiteSpace: 'nowrap', opacity: fade(f - 8), fontVariantNumeric: 'tabular-nums' }}>
          <div><div style={{ fontSize: 25, color: C.sub }}>{t.conFell}</div><div style={{ fontSize: 92, fontWeight: 700, lineHeight: 1.1 }}><span style={{ color: C.red }}>{w5.llm.fell}</span> <span style={{ color: C.text, fontSize: 52 }}>→</span> <span style={{ color: C.green }}>{w5.prev.fell}</span></div></div>
          <div><div style={{ fontSize: 25, color: C.sub }}>{t.conPick}</div><div style={{ fontSize: 92, fontWeight: 700, lineHeight: 1.1 }}>{pct(w5.rule.picked, w5.rule.n)} <span style={{ color: C.text, fontSize: 52 }}>→</span> <span style={{ color: C.green }}>{pct(w5.prev.picked, w5.prev.n)}</span></div></div>
        </div>
        <div style={{ fontSize: 34, lineHeight: 1.45, marginTop: 30, opacity: fade(f - 20) }}>{t.conMain}</div>
        <div style={{ fontSize: 25, color: C.sub, lineHeight: 1.55, marginTop: 26, opacity: fade(f - 36) }}>
          {t.conLim(Math.round(w5.prev.lat_min), Math.round(w5.prev.lat_max), w5.llm.handoff, w5.prev.handoff).map((x) => <div key={x}>{x}</div>)}
        </div>
      </div>
    </Page>
  );
};
