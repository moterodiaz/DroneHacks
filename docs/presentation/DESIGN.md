# Technical presentation: design and text rules

These are the standing rules for Team 08's judging deck. `build_deck.js` follows them. Change them here first, then rebuild.

## Brief

- Audience: two judging panels on 8 Nov, run as a technical interview. The talk is 5–7 min and the questions take 5–6 min.
- Points: 25 for the presentation (autonomy design, technical execution, originality, evidence of testing) and 15 for generality. An AI review of the repository flags hard-coding to the judges.
- Structure: 7 slides covering the autonomy stack, the approach and the results.
  1. Problem: task, platform, constraints.
  2. Autonomy stack.
  3. Approach 1: first principles and the iteration timeline.
  4. Approach 2: planning and the followers.
  5. Approach 3: safety, localization, tools and contracts.
  6. Results 1: flight, SITL and bench results.
  7. Results 2: generality, course-specific items, open items and credits.
- Content: every slide carries technical facts from `technical-corpus/TECHNICAL_CORPUS.md`. Nothing goes on a slide that has no fact in it: no slogans, no taglines and no closing slide.

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
- Each slide has a grey `n/7 SECTION` label, a 22 pt title stating the content (not a slogan), and numbered sub-headings `n.m`.
- Tables are booktabs style: a bold header between black rules, grey hairlines between rows, no vertical lines and no fills.
- Diagrams use square boxes with thin black lines and plain arrows labelled with topic names and rates. Do not use icons, rounded cards, pills, shadows, gradients or decorative stripes.
- Charts are drawn as shapes, because Keynote's PPTX import drops pptxgenjs charts.
- Sizes: body 11.5 pt and tables 10.5 pt on the 13.33 × 7.5 in layout (wide). The source-tag legend is in the footer.

## Process

1. Build: `NODE_PATH=<dir with pptxgenjs> APPLY_THEME=<pptx skill>/scripts/apply_theme.js node docs/presentation/build_deck.js`.
2. Validate with the pptx skill's `validate.py`.
3. Render to PDF through Keynote (`osascript`: open, export as PDF) and look at every page.
4. Do a visual review of each render for four things:
   - **hierarchy**: the title, then the sub-headings, then the body;
   - **contrast**;
   - **alignment**: columns share left edges and tops;
   - **clutter**: remove any line that has no fact.
