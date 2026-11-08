# Technical presentation: design and voice rules

These are the standing rules for Team 08's judging deck, so nobody has to explain them again. `build_deck.js` follows them. Change them here first, then rebuild.

## The brief

- **Audience:** two judging panels, 8 Nov, 12:30–3:10 pm. The talk is 5–7 minutes, then 5–6 minutes of questions, run as a technical interview that probes decisions, experiments, testing and challenges.
- **What is scored:** 25 points for the presentation (autonomy design, technical execution, originality, evidence of testing) and 15 for generality. An AI review of the repository flags course-specific hard-coding and gives the flags to the judges.
- **Length:** at most 8 slides, about 45–50 s each. Detail goes in the speaker notes and the corpus (`technical-corpus/TECHNICAL_CORPUS.md`), not on the slide.

## Voice

- **Each title states the slide's one message**, in 36 pt Arial bold on a single line (about 32 characters at most). No full stop at the end.
- **Plain words, numbers with units.** Write "0.32 m clear of walls", not "a conservative safety buffer".
- **Every number shows where it came from**, with a source chip:
  - **FLOWN**: skybeta-11 logs.
  - **SITL**: our simulator, the real kit code on ported PX4 loops.
  - **MODEL**: offline model flights or tests.
- **Say what is course-specific before the judges do:** the map file, the corridor goal added after the crash, and pathfly's precomputed route and fixed east heading.
- Do not claim results we don't have. The smooth trajectory stack, the return home and pathfly have not flown on the drone.

## Visual system

| Token | Hex | Use |
|---|---|---|
| ink (dk1) | `0E1A2B` | Dark slide backgrounds, body text on light slides, about 60% of the visual weight |
| slate (dk2) | `3D4B5C` | Secondary text |
| paper (lt1) | `FFFFFF` | Light slide backgrounds |
| mist (lt2) | `EEF2F6` | Card fills on light slides |
| signal (accent1) | `F26B1D` | The one sharp accent: our node, the waypoint marks, the FLOWN chip, key numbers |
| steel (accent2) | `3F6E9E` | SITL chip, chart bars other than the highlight |
| go (accent3) | `2BA36B` | Pass, "carries over" |
| ice (accent4) | `C9D6E3` | Secondary text on dark slides, thin rules |
| gray (accent5) | `8A97A6` | MODEL chip, footers |
| stop (accent6) | `D64545` | Fail, "course-specific" |

- **Sandwich:** the title and closing slides are dark (ink); content slides are light (paper).
- **Motif: waypoints.** Steps and lessons are numbered circles: an ink fill, a signal-orange ring and a white number, like the route's numbered waypoints. Use it on every slide that has steps. No accent stripes, bars under titles or edge borders.
- **Fonts:** Arial for headings, Calibri for body (both render true to width in QA). Sizes: titles 36 pt, card headers 15–16 pt, body 14 pt, table text 13 pt, captions and chips 10–11 pt.
- **Layout:** 16:9 at 10 × 5.625 in, 0.5 in margins, 0.3 in gaps between blocks, left-aligned body text. The footer (team · drone · event) and the slide number come from the layout.
- **Charts** are drawn as editable shapes (bars, labels, a baseline), with labelled values, no gridlines, and the highlight bar in signal orange. We don't use a pptxgenjs chart object because Keynote's PPTX import drops it: a minimal test chart exported blank. The team may present from a Mac.

## Process (the Claude deck routine)

1. Content comes from the corpus §§8–12 and §§19–25 and the Drone 11 Flight Stack explainer.
2. Build in batches: slides 1–5, render and review, then slides 6–8.
3. Visual review pass on every render:
   - **hierarchy**: is the title the first thing you read, and the key number the second?
   - **contrast**: is any text below 4.5:1 on its background?
   - **alignment**: do the left edges and the tops of cards line up on a shared grid?
   - **clutter**: is anything on the slide that the speaker notes could carry instead?
4. Validate with the pptx skill's `validate.py`, then render to PDF through Keynote (`osascript`: open, export as PDF) and look at every page.

Build: `NODE_PATH=<dir with pptxgenjs> node docs/presentation/build_deck.js` writes `docs/presentation/DroneHacks-Team08-technical.pptx`.
