import data from './data.json';

export const D: any = data;
export const FPS = 30;
export const W = 1920;
export const H = 1080;

// light theme (LinkedIn)
export const C = {
  bg: '#FFFFFF',
  panel: '#F5F5F3',
  line: '#E2E2DF',
  text: '#141518',
  sub: '#4F5257',
  dim: '#6B6E73',
  red: '#C94A33',
  amber: '#B87A1C',
  green: '#2E8A57',
  blue: '#2F6DB5',
};

export const FONT = 'KoPub Dotum, sans-serif';

export const pct = (a: number, b: number) => `${((100 * a) / b).toFixed(1)}%`;
export const mtok = (n: number) => `${(n / 1e6).toFixed(1)}M`;
export const ktok = (n: number) => `${Math.round(n / 1e3)}k`;
