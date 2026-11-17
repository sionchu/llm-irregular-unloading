import pathlib, shutil, sys
S = pathlib.Path(__file__).parent / 'src'
bak = pathlib.Path(__file__).parent / '_bak_v7'
if not bak.exists():
    shutil.copytree(S, bak)
P = []
def p(f, a, b): P.append((f, a, b))

p('theme.ts', "dim: '#8C8F94'", "dim: '#6B6E73'")
# i18n: meta labels out, one key line per stage
p('i18n.ts', "  why: '모델이 남긴 근거 (원문)',",
  "  why: '모델 판단 근거',\n  stageKey: (r: number) => [`앞에서 당기면 ${r}개 낙하`, '넘김 18회 · 너무 많이 넘김', '받아 내도 낙하는 그대로', '집음 81% · 낙하 56', '낙하 56 → 8'],")
p('i18n.ts', "  why: 'Model reasoning (verbatim)',",
  "  why: 'Model reasoning',\n  stageKey: (r) => [`Pulled from front: ${r} fell`, 'Handed off 18 times', 'Catching does not stop falls', '81% picked · 56 fell', 'Falls 56 → 8'],")
p('i18n.ts', "받침 관계 (시뮬 상태)'", "받침 관계'")
p('i18n.ts', "what rests on what (sim state)'", "what rests on what'")
p('i18n.ts', " (출력 ${out} 토큰, 입력 토큰 기록 없음)`", " · 출력 ${out} 토큰`")
p('i18n.ts', " (${out} output tokens; input not logged)`", " · ${out} output tokens`")
p('i18n.ts', "GPT-6 Astra (Codex, ChatGPT 요금제).", "GPT-6 Astra.")
p('i18n.ts', "GPT-6 Astra (Codex, ChatGPT plan).", "GPT-6 Astra.")
# ui: Korean line breaks, filled active step, filled active chip, smaller model name, band dimming
p('ui.tsx', "fontFamily: FONT, fontWeight: 500 }}", "fontFamily: FONT, fontWeight: 500, wordBreak: 'keep-all' }}")
p('ui.tsx', """<div key={s} style={{ width: 236, marginRight: 14, borderTop: `4px solid ${!used[i] ? C.line : i === active ? C.blue : '#C9CACC'}`, paddingTop: 6,
            opacity: used[i] ? 1 : 0.4 }}>
            <span style={{ fontSize: 27, fontWeight: 700, color: i === active ? C.text : used[i] ? C.sub : C.dim }}>{s}</span>
            <span style={{ fontSize: 17, color: C.dim, marginLeft: 8 }}>{used[i] ? t.stepSub[i] : t.unused}</span>""",
"""<div key={s} style={{ width: 272, marginRight: 12, borderTop: `4px solid ${!used[i] ? C.line : i === active ? C.blue : '#C9CACC'}`, padding: '4px 10px 5px',
            background: i === active ? C.blue : 'transparent', opacity: used[i] ? 1 : 0.4, whiteSpace: 'nowrap', overflow: 'hidden' }}>
            <span style={{ fontSize: 28, fontWeight: 700, color: i === active ? '#FFFFFF' : used[i] ? C.sub : C.dim }}>{s}</span>
            <span style={{ fontSize: 19, color: i === active ? '#FFFFFF' : C.dim, marginLeft: 8 }}>{used[i] ? t.stepSub[i] : t.unused}</span>""")
p('ui.tsx', """      background: strong ? C.panel : C.bg, fontSize: 23, whiteSpace: 'nowrap', color: C.text }}>
      <span style={{ width: 10, height: 10, background: chipColor(c), display: 'inline-block' }} />""",
"""      background: strong ? chipColor(c) : C.bg, fontSize: strong ? 26 : 23, fontWeight: strong ? 700 : 500, whiteSpace: 'nowrap', color: strong ? '#FFFFFF' : C.text }}>
      <span style={{ width: 10, height: 10, background: strong ? '#FFFFFF' : chipColor(c), display: 'inline-block' }} />""")
p('ui.tsx', "calls.slice(-5).map", "calls.slice(-3).map")
p('ui.tsx', "fontSize: 42, fontWeight: 700, marginTop: 4, lineHeight: 1.15 }}>{rule ? '—' : model}",
  "fontSize: 32, fontWeight: 700, marginTop: 6, lineHeight: 1.15 }}>{rule ? '—' : model}")
p('ui.tsx', "decided?: boolean; rule?: boolean; tag?: string;\n}> = ({ model, used, active, calls, rec, decided, rule, tag }) => {",
  "decided?: boolean; rule?: boolean; tag?: string; dimTo?: number;\n}> = ({ model, used, active, calls, rec, decided, rule, tag, dimTo = 1 }) => {")
p('ui.tsx', "height: BAND_H, borderBottom: `1px solid ${C.line}`, boxSizing: 'border-box', padding: '14px 48px 0' }}>",
  "height: BAND_H, borderBottom: `1px solid ${C.line}`, boxSizing: 'border-box', padding: '14px 48px 0', opacity: dimTo }}>")
p('ui.tsx', "<div style={{ width: 300, fontSize: 22, color: C.sub, paddingTop: 6 }}>{tag}</div>",
  "<div style={{ width: 300, fontSize: 22, color: C.sub, paddingTop: 8 }}>{tag}</div>")
# stages: one big key line over the video
p('Stages.tsx', """textShadow: '0 0 6px rgba(0,0,0,0.5)' }}>×{rate.toFixed(1)}</div>""",
"""textShadow: '0 0 6px rgba(0,0,0,0.5)' }}>×{rate.toFixed(1)}</div>
      <div style={{ position: 'absolute', left: 48 + 24, top: top + 24, padding: '10px 22px', background: vc, color: '#FFFFFF', fontSize: 44, fontWeight: 700,
        boxShadow: '0 2px 12px rgba(0,0,0,0.2)', transformOrigin: 'left top',
        transform: `scale(${interpolate(f, [15, 21, 25], [0.8, 1.06, 1], { extrapolateLeft: 'clamp', extrapolateRight: 'clamp' })})`, opacity: interpolate(f, [15, 30], [0, 1], { extrapolateLeft: 'clamp', extrapolateRight: 'clamp' }) }}>{t.stageKey(R29)[s.n - 1]}</div>""")
# grid: band dimmed, fell numbers big
p('Grid.tsx', "<LlmBand tag={t.gridBand(ev?.t)}", "<LlmBand dimTo={f >= RES_AT ? 0.5 * (1 - fade(f - RES_AT, 10)) : 0.5} tag={t.gridBand(ev?.t)}")
p('Grid.tsx', """display: 'flex', gap: 24, fontSize: 27,
              fontVariantNumeric: 'tabular-nums' }}>
              <span>{t.picked} <b style={{ color: C.green }}>{last?.picked ?? 0}</b><span style={{ color: C.dim }}>/44</span></span>
              <span>{t.fell} <b style={{ color: C.red }}>{last?.fell ?? 0}</b></span>
              <span>{t.handed} <b style={{ color: C.amber }}>{last?.handed ?? 0}</b></span>""",
"""display: 'flex', alignItems: 'baseline', gap: 24, fontSize: 24, color: C.sub,
              fontVariantNumeric: 'tabular-nums' }}>
              <span style={{ fontSize: 40, fontWeight: 700, color: C.red }}>{t.fell} {last?.fell ?? 0}</span>
              <span>{t.picked} <b style={{ color: C.green }}>{last?.picked ?? 0}</b>/44</span>
              <span>{t.handed} <b style={{ color: C.amber }}>{last?.handed ?? 0}</b></span>""")
p('Grid.tsx', "<div style={{ position: 'absolute', left: 48, right: 48, top, bottom: 30, background: 'rgba(255,255,255,0.92)',",
  "<div style={{ position: 'absolute', left: 48, right: 48, top: 30, bottom: 30, background: '#FFFFFF',")
# results: winning row outlined, key cells bigger, source notes out
p('Cards.tsx', "        <div style={{ color: C.dim }}>{t.resNote}</div>\n", "")
p('Cards.tsx', "background: i === 3 ? C.panel : 'transparent' }}>", "background: i === 3 ? '#EEF6F1' : 'transparent', outline: i === 3 ? `3px solid ${C.green}` : 'none' }}>")
p('Cards.tsx', "                {pct(r.picked, r.n)}\n", "                <span style={i === 3 ? { fontSize: 38, fontWeight: 700, color: C.green } : undefined}>{pct(r.picked, r.n)}</span>\n")
p('Cards.tsx', "color: r.fell <= 10 ? C.green : C.red, fontWeight: 700 }}>", "color: r.fell <= 10 ? C.green : C.red, fontWeight: 700, fontSize: i === 3 ? 40 : undefined }}>")
# ending: numbers dominate
p('Cards.tsx', "fontSize: 72, fontWeight: 700, marginTop: 4, opacity: fade(f) }}>{t.conTitle}", "fontSize: 58, fontWeight: 700, marginTop: 4, opacity: fade(f) }}>{t.conTitle}")
p('Cards.tsx', "<div style={{ fontSize: 58, fontWeight: 700 }}><span style={{ color: C.red }}>", "<div style={{ fontSize: 92, fontWeight: 700, lineHeight: 1.1 }}><span style={{ color: C.red }}>")
p('Cards.tsx', "<div style={{ fontSize: 58, fontWeight: 700 }}>{pct(", "<div style={{ fontSize: 92, fontWeight: 700, lineHeight: 1.1 }}>{pct(")
p('Cards.tsx', "<span style={{ color: C.dim, fontSize: 36 }}>→</span> <span style={{ color: C.green }}>", "<span style={{ color: C.text, fontSize: 52 }}>→</span> <span style={{ color: C.green }}>")
p('Cards.tsx', "<span style={{ color: C.dim, fontSize: 36 }}>→</span> {pct(", "<span style={{ color: C.text, fontSize: 52 }}>→</span> <span style={{ color: C.green }}>{pct(")
p('Cards.tsx', "{pct(w5.prev.picked, w5.prev.n)}</div></div>", "{pct(w5.prev.picked, w5.prev.n)}</span></div></div>")
# hook: labels readable
p('Cards.tsx', "<div style={{ fontSize: 28, fontWeight: 700, color: k === 0 ? C.sub : C.text }}>", "<div style={{ fontSize: 38, fontWeight: 700, color: C.text }}>")

txt = {}
bad = 0
for f, a, b in P:
    s = txt.get(f) or (S / f).read_text(encoding='utf-8')
    n = s.count(a)
    if n != 1:
        print('MISS', n, f, a[:70].replace('\n', ' ')); bad += 1; continue
    txt[f] = s.replace(a, b)
if bad:
    sys.exit(1)
for f, s in txt.items():
    (S / f).write_text(s, encoding='utf-8')
print('ok', len(P))
