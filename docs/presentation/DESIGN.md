# Technical presentation: design and text rules

These are the standing rules for Team 08's judging deck. `build_deck.js` follows them. Change them here first, then rebuild.

## Brief

- Audience: two judging panels on 8 Nov, run as a technical interview. The talk is 5–7 min and the questions take 5–6 min.
- Points: 25 for the presentation (autonomy design, technical execution, originality, evidence of testing) and 15 for generality. An AI review of the repository flags hard-coding to the judges.
- Structure: a research talk. A title slide, then 8 slides:
  0. Title: team, drone, flown package and its SITL result, route figure from `route_fig.py`.
  1. Problem: task, scoring, constraints.
  2. System: data flow and control loops.
  3. Method: planning (map in, route out) and the followers.
  4. Method: safety guards and verification tools.
  5. Experiments: the iterative flight-test timeline.
  6. Results: flown and SITL results.
  7. Generality: planner vs hand route, declared course-specific items, new-course procedure, credits.
  8. Limitations and future work, with the speed ladder.
- Slides are recall cues, not the script. Each block has 3–5 bullets of at most about 8 words. Each slide has one visual (diagram, figure, table or bars). All detail goes in the speaker notes.
- Content: every slide carries technical facts from `technical-corpus/TECHNICAL_CORPUS.md`. Nothing goes on a slide that has no fact in it: no slogans, no taglines and no closing slide. Rehearsal questions and the do-not-claim list go in the speaker notes.

## Text: ASD-STE100 (Simplified Technical English)

- One topic per sentence. Procedural sentences have 20 words at most; descriptive sentences 25 at most.
- Use the active voice. Use the simple present tense for facts and the simple past tense for test results.
- Write instructions as commands ("Tape the launch mark."). Write "do not", not "avoid".
- Use one term for one thing: drone, setpoint, map file, launch mark, stop-and-go, trajectory stack, pathfly.
- No idioms, no marketing adjectives and no rhetorical questions.
- Every number has a unit and a source tag:
  - **[F]**: flown on skybeta-11.
  - **[S]**: our SITL.
  - **[M]**: model, bench or test.
- State course-specific items and items not yet flown on the slide itself.

## Visual style: engineering report

- White background with black and dark grey text. Arial throughout; Courier New for the formula and the topic labels.
- Two accents only:
  - red `B42318` for failures and contacts;
  - dark blue `1F3A5F` for diagram topic labels, the best SITL result and step numbers.
- Each content slide has a grey `n  SECTION` label, a 28 pt title that states the finding (not a slogan), and short unnumbered sub-headings.
- Tables are booktabs style: a bold header between black rules, grey hairlines between rows, no vertical lines and no fills.
- Diagrams use square boxes with thin black lines and plain arrows labelled with topic names and rates. Do not use icons, rounded cards, pills, shadows, gradients or decorative stripes.
- Charts are drawn as shapes, because Keynote's PPTX import drops pptxgenjs charts.
- Sizes: bullets 17–18 pt, sub-headings 17 pt, tables 15 pt, nothing on a slide below 11 pt, on the 13.33 × 7.5 in layout (wide). The source-tag legend is in the footer.

## Process

1. Build: `NODE_PATH=<dir with pptxgenjs> APPLY_THEME=<pptx skill>/scripts/apply_theme.js node docs/presentation/build_deck.js`.
2. Route figure: `python docs/presentation/route_fig.py <follower package dir> docs/presentation/assets/route-<sha>.png` plots the planner's own route on the map file.
3. Validate with the pptx skill's `validate.py`.
4. Render to PDF through Keynote (`osascript`: open, export as PDF) and look at every page.
5. Do a visual review of each render for four things:
   - **hierarchy**: the title, then the sub-headings, then the body;
   - **contrast**;
   - **alignment**: columns share left edges and tops;
   - **clutter**: remove any line that has no fact.
