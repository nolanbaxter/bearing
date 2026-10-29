// The README's engine comparison as a slideshow: one animated SVG, a chart a
// slide, each held SLIDE_SECONDS with a short fade, dots to say which.
// Winding's tools/compare-chart.mjs, the same layout and colours, so the two
// READMEs read alike: grouped by scene, a colour per engine, each slide its own
// axis. Writes docs/images/compare-light.svg and compare-dark.svg. The numbers
// are bench/RESULTS.md's, from bench/compare.html.
//
// Run: node bench/chart.mjs docs/images
//
// A slide is { title, note, cap, ticks, groups, values }: values[engine][group].
// More comparisons are more slides; the size stays 760 x HEIGHT whatever the count.
import { writeFileSync } from 'node:fs';

const OUT = process.argv[2];
const SLIDE_SECONDS = 7;
const FADE = 0.4;   // seconds
const engines = ['Bearing', 'Bearing (1 thread)', 'Rapier', 'Jolt', 'cannon-es'];
const scenes = ['1,000 boxes landing', '1,000 hulls landing', '32 ragdolls', '210-box pyramid'];
const slides = [
  {
    title: 'Time per step, ms (lower is better)',
    note: 'Laptop: Intel i7-1260P, Chrome 152',
    cap: 30, ticks: [0, 10, 20, 30], groups: scenes,
    // bench/RESULTS.md, 2026-10-29 (25db6d7): the piles' first 5 s, the ragdolls' first 4 s.
    values: {
      'Bearing': [6.59, 15.0, 7.73, 0.411],
      'Bearing (1 thread)': [12.6, 19.7, 7.65, 0.415],
      'Rapier': [2.22, 3.52, 1.17, 0.0767],
      'Jolt': [5.45, 8.03, 2.93, 0.0746],
      'cannon-es': [21.0, 269, 5.96, 6.78],
    },
  },
];
// Winding's four series colours, Bearing first in Winding's blue; the second
// Bearing a lighter tint of it, so the pair reads as one engine two ways.
const themes = {
  light: { series: ['#2a78d6', '#8fbcef', '#eb6834', '#1baf7a', '#eda100'], ink: '#0b0b0b', secondary: '#52514e', muted: '#6b6a65', grid: '#e1e0d9', base: '#c3c2b7', page: '#ffffff', dot: '#c3c2b7' },
  dark: { series: ['#3987e5', '#22508a', '#d95926', '#199e70', '#c98500'], ink: '#f0efec', secondary: '#c3c2b7', muted: '#9a9891', grid: '#2c2c2a', base: '#4a4a46', page: '#0d1117', dot: '#4a4a46' },
};

const W = 760, HEIGHT = 400, left = 52, right = 740, plotTop = 110, bot = 330;
const fmt = (v) => v.toFixed(v < 0.1 ? 3 : v < 1 ? 2 : v < 10 ? 1 : 0);
const font = `font-family="-apple-system, 'Segoe UI', Helvetica, Arial, sans-serif"`;

/** One slide's chart, in theme `t`. */
function slide(sl, t) {
  const yOf = (v) => bot - Math.min(v, sl.cap) / sl.cap * (bot - plotTop);
  let s = `<text x="${left - 40}" y="26" font-size="15" font-weight="600" fill="${t.ink}">${sl.title}</text>\n`;
  s += `<text x="${left - 40}" y="48" font-size="13" fill="${t.secondary}">${sl.note}</text>\n`;
  // Legend: a square of colour beside each name; the names wear text ink.
  let lx = left - 40;
  engines.forEach((e, i) => {
    s += `<rect x="${lx}" y="62" width="11" height="11" rx="2" fill="${t.series[i]}"/><text x="${lx + 17}" y="72" font-size="13" fill="${t.secondary}">${e}</text>\n`;
    lx += 17 + e.length * 7.4 + 22;
  });
  for (const v of sl.ticks) {
    s += `<line x1="${left}" x2="${right}" y1="${yOf(v)}" y2="${yOf(v)}" stroke="${t.grid}" stroke-width="1"/>`;
    s += `<text x="${left - 8}" y="${yOf(v) + 4}" font-size="12" text-anchor="end" fill="${t.muted}">${v}</text>\n`;
  }
  // Five engines to Winding's four, so the bars are narrower: 4 groups of 5
  // fit 688 px with room between groups.
  const gw = (right - left) / sl.groups.length, bw = 26, gap = 3, w = engines.length * bw + (engines.length - 1) * gap;
  sl.groups.forEach((group, gi) => {
    const x0 = left + gi * gw + (gw - w) / 2;
    engines.forEach((e, ei) => {
      const v = sl.values[e][gi], x = x0 + ei * (bw + gap), yt = yOf(v);
      s += `<path d="M${x},${bot} V${yt + 4} q0,-4 4,-4 h${bw - 8} q4,0 4,4 V${bot} Z" fill="${t.series[ei]}"/>`;
      // A bar past the axis is cut, and says so: a break across it, its real value above.
      if (v > sl.cap) s += `<path d="M${x - 2},${yt + 20} l${bw + 4},-8 v6 l-${bw + 4},8 Z" fill="${t.page}"/>`;
      s += `<text x="${x + bw / 2}" y="${yt - 6}" font-size="11" text-anchor="middle" fill="${t.ink}">${fmt(v)}</text>\n`;
    });
    s += `<text x="${left + gi * gw + gw / 2}" y="${bot + 24}" font-size="13" text-anchor="middle" fill="${t.secondary}">${group}</text>\n`;
  });
  s += `<line x1="${left}" x2="${right}" y1="${bot}" y2="${bot}" stroke="${t.base}" stroke-width="1"/>\n`;
  s += `<text x="${left}" y="${HEIGHT - 16}" font-size="11" fill="${t.muted}">Axis cut at ${sl.cap} ms: a bar that passes it is broken, its value above.</text>\n`;
  return s;
}

const n = slides.length, cycle = n * SLIDE_SECONDS;
const pct = (seconds) => `${(seconds / cycle * 100).toFixed(3)}%`;
// Each slide fades in, holds, and fades out in its own SLIDE_SECONDS of the cycle.
const keyframes = `@keyframes show { 0% { opacity: 0 } ${pct(FADE)} { opacity: 1 } ${pct(SLIDE_SECONDS - FADE)} { opacity: 1 } ${pct(SLIDE_SECONDS)} { opacity: 0 } 100% { opacity: 0 } }`;

for (const [name, t] of Object.entries(themes)) {
  let s = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${HEIGHT}" width="${W}" height="${HEIGHT}" role="img" ${font}>\n`;
  s += `<title>Bearing against Rapier, Jolt and cannon-es${n > 1 ? ', a chart at a time' : ''}</title>\n`;
  s += `<desc>${slides.map((sl) => `${sl.title}, ${sl.note}: ${sl.groups.map((g, i) => `${g}: ${engines.map((e) => `${e} ${fmt(sl.values[e][i])}`).join(', ')}`).join('. ')}`).join('. ')}.</desc>\n`;
  if (n > 1) {
    s += `<style>\n.slide { opacity: 0; animation: show ${cycle}s linear infinite; }\n${keyframes}\n`;
    s += `.dot { fill: ${t.dot}; } .on { fill: ${t.secondary}; opacity: 0; animation: show ${cycle}s linear infinite; }\n`;
    slides.forEach((_, i) => { s += `.s${i} { animation-delay: ${i * SLIDE_SECONDS}s; }\n`; });
    // Asked for less motion: the first chart, held.
    s += `@media (prefers-reduced-motion: reduce) { .slide, .on { animation: none; } .s0 { opacity: 1; } }\n</style>\n`;
  }
  // One slide is simply shown: there is nothing to turn to.
  slides.forEach((sl, i) => { s += `<g${n > 1 ? ` class="slide s${i}"` : ''}>\n${slide(sl, t)}</g>\n`; });
  // Which slide: a dot each, the current one lit.
  if (n > 1) {
    slides.forEach((_, i) => {
      const cx = right - (n - 1 - i) * 18;
      s += `<circle class="dot" cx="${cx}" cy="${HEIGHT - 20}" r="4"/><circle class="on s${i}" cx="${cx}" cy="${HEIGHT - 20}" r="4"/>\n`;
    });
  }
  s += `</svg>\n`;
  writeFileSync(`${OUT}/compare-${name}.svg`, s);
}
console.log(`written: ${n} slide${n > 1 ? 's' : ''}${n > 1 ? `, ${SLIDE_SECONDS} s each` : ''}`);
