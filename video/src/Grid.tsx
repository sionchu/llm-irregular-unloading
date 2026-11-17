import React from 'react';
import { useCurrentFrame } from 'remotion';
import { C, D, FPS } from './theme';
import { useT } from './i18n';
import { Page, Clip, clipInfo, simFrame, eventsAt, LlmBand, BAND_H, revealCalls, Call } from './ui';
import { ResultsPanel, fade } from './Cards';

const CELLS = [
  { clip: 'grid_rule300', llm: false },
  { clip: 'grid_rulelift300', llm: false },
  { clip: 'grid_astralift300', llm: true },
  { clip: 'grid_astraprev300', llm: true },
];
const PLAY_S = 30;
export const gridRate = () => Math.max(1, Math.max(...CELLS.map((c) => clipInfo(c.clip)?.frames ?? 900)) / (PLAY_S * FPS));
const RES_AT = (PLAY_S + 2) * FPS; // results panel slides over the frozen grid
export const GRID_DUR = RES_AT + 14 * FPS;

export const GridScene: React.FC = () => {
  const f = useCurrentFrame();
  const t = useT();
  const rate = gridRate();
  const gap = 12;
  const top = BAND_H + 16;
  const ch = Math.floor((1080 - top - 30 - gap) / 2);
  const cw = Math.floor((ch * 16) / 9);
  const pc = 'grid_astraprev300';
  const psf = simFrame(pc, f, rate);
  const pe = eventsAt(pc, psf);
  const ev: any = pe.cur ?? pe.last;
  let calls: Call[] = [];
  let active = -1;
  let rec: any = null;
  let decided = false;
  if (ev) {
    rec = (D.trace.prevAll as any[]).find((r) => r.t === ev.t);
    const u = pe.cur ? (psf - ev.f0) / Math.max(1, ev.f1 - ev.f0) : 1;
    const r = revealCalls(rec?.calls ?? [], u);
    calls = r.shown; active = r.active; decided = r.decided;
  }
  const speed = Math.round(rate * 4); // sim frames are every 8th 60 Hz update -> 7.5 fps of sim time
  const side = t.gridSide(speed);
  return (
    <Page>
      <LlmBand dimTo={f >= RES_AT ? 0.5 * (1 - fade(f - RES_AT, 10)) : 0.5} tag={t.gridBand(ev?.t)} model="GPT-6 Astra" used={[true, true, true, true, true]} active={active} calls={calls} rec={rec} decided={decided} />
      {CELLS.map((c, i) => {
        const x = 48 + (i % 2) * (cw + gap);
        const y = top + Math.floor(i / 2) * (ch + gap);
        const { last } = eventsAt(c.clip, simFrame(c.clip, f, rate));
        const info = clipInfo(c.clip);
        const ended = info ? f * rate >= info.frames - 1 : false;
        const hl = i === 3;
        return (
          <div key={c.clip} style={{ position: 'absolute', left: x, top: y, width: cw, height: ch, outline: hl && f < RES_AT ? `3px solid ${C.text}` : 'none' }}>
            <Clip name={c.clip} rate={rate} style={{ width: cw, height: ch }} />
            <div style={{ position: 'absolute', left: 0, top: 0, padding: '6px 12px', background: 'rgba(255,255,255,0.92)', fontSize: 27, fontWeight: 700,
              color: c.llm ? C.text : C.sub }}>{t.gridCells[i]}</div>
            <div style={{ position: 'absolute', left: 0, bottom: 0, right: 0, padding: '8px 12px', background: 'rgba(255,255,255,0.92)', display: 'flex', alignItems: 'baseline', gap: 24, fontSize: 24, color: C.sub,
              fontVariantNumeric: 'tabular-nums' }}>
              <span style={{ fontSize: 40, fontWeight: 700, color: C.red }}>{t.fell} {last?.fell ?? 0}</span>
              <span>{t.picked} <b style={{ color: C.green }}>{last?.picked ?? 0}</b>/44</span>
              <span>{t.handed} <b style={{ color: C.amber }}>{last?.handed ?? 0}</b></span>
              {ended && <span style={{ marginLeft: 'auto', color: C.dim }}>{t.ended}</span>}
            </div>
          </div>
        );
      })}
      <div style={{ position: 'absolute', left: 48 + 2 * cw + gap + 36, top, right: 48, fontSize: 24, color: C.sub, lineHeight: 1.5, opacity: 1 - fade(f - RES_AT, 8) }}>
        <div style={{ fontSize: 30, color: C.text, fontWeight: 700 }}>{side[0]}</div>
        <div style={{ marginTop: 8 }}>{side[1]}</div>
        <div style={{ marginTop: 22 }}>{side[2]}</div>
      </div>
      {f >= RES_AT && (
        <div style={{ position: 'absolute', left: 48, right: 48, top: 30, bottom: 30, background: '#FFFFFF', border: `1px solid ${C.line}`,
          opacity: fade(f - RES_AT, 10), transform: `translateY(${(1 - fade(f - RES_AT, 12)) * 24}px)` }}>
          <ResultsPanel f={f - RES_AT} />
        </div>
      )}
    </Page>
  );
};
