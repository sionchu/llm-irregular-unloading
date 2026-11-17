import React from 'react';
import { Composition, Series } from 'remotion';
import { FPS, W, H } from './theme';
import { useFonts } from './ui';
import { LangCtx, Lang } from './i18n';
import { STAGES, StageScene, stageDur } from './Stages';
import { GridScene, GRID_DUR } from './Grid';
import { Hook, HOOK_DUR, Ending, END_DUR } from './Cards';

// every scene sits on simulation footage: no standalone explainer cards
const PARTS: [string, number, React.FC][] = [
  ['hook', HOOK_DUR, Hook],
  ...STAGES.map((s) => [`stage${s.n}`, stageDur(s), () => <StageScene s={s} />] as [string, number, React.FC]),
  ['grid', GRID_DUR, GridScene],
  ['ending', END_DUR, Ending],
];
const TOTAL = PARTS.reduce((a, p) => a + p[1], 0);

const Main: React.FC<{ lang: Lang }> = ({ lang }) => {
  useFonts();
  return (
    <LangCtx.Provider value={lang}>
      <Series>
        {PARTS.map(([k, d, Comp]) => (
          <Series.Sequence key={k} durationInFrames={d}>
            <Comp />
          </Series.Sequence>
        ))}
      </Series>
    </LangCtx.Provider>
  );
};

const Single: React.FC<{ part: string; lang: Lang }> = ({ part, lang }) => {
  useFonts();
  const Comp = PARTS.find((p) => p[0] === part)![2];
  return (
    <LangCtx.Provider value={lang}>
      <Comp />
    </LangCtx.Provider>
  );
};

export const Root: React.FC = () => (
  <>
    {(['ko', 'en'] as Lang[]).map((lang) => (
      <React.Fragment key={lang}>
        <Composition id={`Main-${lang}`} component={Main as any} defaultProps={{ lang }} durationInFrames={TOTAL} fps={FPS} width={W} height={H} />
        {PARTS.map(([k, d]) => (
          <Composition key={k} id={`${lang}-${k}`} component={Single as any} defaultProps={{ part: k, lang }} durationInFrames={d} fps={FPS} width={W} height={H} />
        ))}
      </React.Fragment>
    ))}
  </>
);
