# Editorial Changelog — JHSS Manuscript

**Paper:** *Design and Validation of a Thrust Vector Controlled Model Rocket Using PID-Kalman Architecture*
**Author:** Marc Julian Dackiw
**Scope of this record:** every change made while working through the ~90 hand-written annotations on the review PDF, organized into four sequential passes plus the apogee rescope and the additive (ADD) judgment calls.

---

## How to read this

Each entry lists the **location**, the **annotation number**, a **category tag**, the **change** (exact before → after for short edits; a description for full-paragraph rewrites), and the **reason**. Category tags:

- **Scope** — the apogee/barometer rescope
- **Correctness** — factual or logical fix
- **Voice** — rewrite to match the author's register
- **Define** — a term given a plain-language gloss
- **Expand** — added explanation grounded in what was actually done
- **Word** — single word or short phrase swap
- **Add** — new content from an ADD judgment call

Three principles governed the work and are worth recording, because they explain why several edits went the way they did:

1. **Claims match the work that was actually done.** Where an annotation invited language the project couldn't support (the nose cone "optimization," apogee detection without a barometer), the text was corrected to reality rather than inflated to fit the word.
2. **Numbers and methods are grounded before prose describes them.** The 0.025 N·m disturbance torque and the sensitivity-sweep method were confirmed against the figures code before being explained in the text.
3. **One unified voice, not uniform flattening.** Voice rewrites removed register breaks (rhythmic em-dashes, colon-introduced lists, three-item-list sentence structures, over-polished phrasing) while preserving precision. "While" was rationed rather than used as a default connector.

---

## Cross-cutting decision: the apogee / barometer rescope

The manuscript originally described apogee detection using barometric altitude. The vehicle that was actually built and validated is the Revision 1 test-stand board, which has **no barometer**, because a ground-constrained two-degree-of-freedom rig never changes altitude. The altitude/apogee language was a fossil from the paper's original free-flight scope, left uncorrected when the project pivoted to simulation-and-stand validation.

**Resolution (Option A):** the flight-software architecture is kept in full, but every altitude-dependent phase (apogee detection, descent, landing) is now framed explicitly as the **intended free-flight design**, not as something the validated hardware executes. A free-flight revision would carry a barometer; the test-stand board does not, and the text now says so. This affects §2.3, §4.5, §4.6, and Appendix A.2 together (edits A1–A10 below).

---

## Changes by section

### Abstract / §1 Introduction

- **§1, V1 — Voice (#4).** The claim that the paper documents the system "in full mathematical and software detail" was softened to "in enough detail to serve as an educational and reproducible reference," removing an overclaim.
- **§1, #5 — Word.** "the ultimate objective of this **research**" → "this **work**" (applied by author), a better fit for a design-and-validation study.

### §2 System Architecture

- **§2.1, nose cone — Expand/Correctness (#8).** *Before:* "Its geometry was **optimized** to minimize frontal area and promote stable airflow…" *After:* "Its profile follows a conventional low-drag form, shaped to keep frontal area small…" The shape was designed by judgment, not by software optimization, so "optimized" was removed rather than backed with invented method.
- **§2.1, B7 — Correctness (#9).** The motor mount sentence claimed high infill density "while minimizing mass," which is contradictory (dense infill adds mass). Rewritten so the added mass is framed as an accepted tradeoff for a stiff mount that keeps the center of mass fixed.
- **§2.1, E3 — Expand (#11).** Clarified what "unintended moments" means: an offset between the thrust line and the center of mass produces a steady turning moment even at neutral gimbal, which the controller must continuously cancel.
- **§2.1, #10 — Word.** "due to its **predictable** thrust curve" → "**well-characterized**" (NAR-certified data).
- **§2.3, A1 — Scope/Define (#16, #17, #18, #19).** Sensor-limitations paragraph rewritten: explains that the missing magnetometer leaves the roll axis without a heading reference, and that the missing barometer reflects the test-stand scope (a free-flight version would carry one for apogee detection). Also folds in the I²C and "low-dynamic conditions" glosses (see D1).
- **§2.3, A2 — Scope (#20, #21).** Logging paragraph: removed "apogee detection events" from the logged parameters, changed the vague "between 100 and 200 Hz" to "the loop rate," and reframed "post-flight analysis" as "analysis during testing."
- **§2.3, D1 — Define (#16, #17).** "low-dynamic conditions" defined as when the vehicle is not accelerating hard, so gravity dominates the accelerometer reading; I²C defined as a standard two-wire serial bus.
- **§2.4, D3 — Define (#24).** "without risk of timing overruns" → "without the loop ever running long enough to miss its scheduled update."
- **§2.5, V2 — Voice (#29, #30).** The inverted-pendulum paragraph (flagged as an "add-on" in a different register) was rewritten to plain prose and now points to §3.1 for how the control loop supplies the required correction.
- **§2.5, #28 — Add.** Appended a sentence on the limits of TVC authority: it comes entirely from the motor, so the gimbal can correct only while the motor burns, with authority that rises and falls with the thrust curve. Balances a section that previously only sold TVC's advantages.
- **§2.6, V3 — Voice (#32).** Thermal-protection paragraph de-stiffened ("was a primary concern due to the proximity of" → "mattered because the avionics sit close to the motor").
- **§2.6, V4 — Voice (#33).** Vibration paragraph brought down to the §2.5 register.
- **§2.6, D2 — Define (#34).** "coupling effects" defined in context: a thrust-line offset produces an unintended moment the controller must spend authority correcting.
- **§2.6, E4 — Expand (#38).** Added an explanation of what static margin is (the CoP–CoM gap in body diameters, with sign deciding stability) and why the negative value here means tilt grows rather than corrects.
- **§2.6, E5 — Expand (#39).** Drew out the implication of flying unstable: stability depends entirely on the control system, and the vehicle stays upright only while the gimbal out-torques the aerodynamic moment. Also removed a colon.

### §3 Control Theory

- **§3.1, B3 — Correctness (#49).** Removed "coast-phase corrections" as a use case for the integral term — during unpowered coast the gimbal has no thrust and therefore no control authority. Reframed to longer powered burns.
- **§3.1, D4 — Define (#51).** "Integral windup" defined at its first mention (the integral term accumulating while the gimbal is already saturated); this also repaired a dangling "described above" that pointed at a definition that wasn't there.
- **§3.1, E6 — Expand (#47).** Linked the derivative term's noise sensitivity to the reason the loop depends on the Kalman filter, which smooths the estimate before differentiation.
- **§3.1, E7 — Expand (#50).** Explained how the Ziegler–Nichols method works (raise proportional gain to sustained oscillation, then set the three terms from the critical gain and period) and named the proportional, integral, and derivative terms.
- **§3.2, V5 — Voice (#52, #53, #54).** Sensor-fusion opening rewritten: added "to act on," trimmed the basic-calculus drift explanation, and de-polished the colon-introduced "the gyroscope over short timescales, the accelerometer over long ones."
- **§3.2, E8 — Expand (#63).** Added why roll drifts: integrating the gyroscope's small errors over time lets them accumulate with nothing to correct them.
- **§3.2, E9 — Expand (#64).** Gave the 70–80% noise-reduction figure its significance (it matters most for the derivative term), its honest limitation (a simulation result from datasheet noise), and where further gains would come from (mechanical isolation or a lower-noise sensor).
- **§3.4 opening, V6 — Voice (#68, #69).** Fixed the confusing first sentence and the dumped-list "bounded by several factors: servo speed, motor thrust…" sentence; removed both colons. (This was the calibration sample.)
- **§3.4, B4 — Correctness (#70).** Reframed the equation lead-in as "the rotational form of Newton's second law, the angular analog of F = ma," answering why the rotational form is used.
- **§3.4.2, B2 — Correctness (#76).** Deleted a duplicated sentence ("At small tilt angles, the aerodynamic torque scales approximately linearly with the tilt"), which appeared twice; the fuller version one paragraph below was kept.
- **§3.4.2, B5 — Correctness (#81).** Added that θ (vehicle tilt) is distinct from δ (gimbal deflection) where the two appear together.
- **§3.4.2, B6 — Correctness (#84).** Named the θ_max numerator explicitly as τ_max = F_T · r · sin(δ_max), so the reader sees it is the §3.4.1 result reused.
- **§3.4.2, V7 — Voice (#75).** The "dominant disturbance" paragraph rewritten to remove the colon and formal register (applied after B2).
- **§3.4.2, D5 — Define (#82).** C_Nα defined as the normal-force-coefficient slope, setting how steeply the aerodynamic side force grows with angle of attack.
- **§3.4.3, V8 — Voice (#87).** "as lightweight vehicles often do" → "which lightweight vehicles are especially prone to."
- **§3.4.3, V9 — Voice (#89).** The "Two design choices mitigate this risk:" colon-list rewritten into two plain sentences.

### §4 Flight Software

- **§4.1, V10 — Voice (#94, #95).** Gave examples of the "derived kinematic quantities" (integrated velocity, tilt angle) and de-stiffened the "ensuring that" sentence. *(Author's edit retained: "or has landed" and "when the system deems it necessary," keeping the fuller register.)*
- **§4.5, A3–A5 — Scope (#103).** The three apogee paragraphs reframed as the intended free-flight design: apogee detection belongs to the free-flight vehicle, the altitude condition is tied to the barometer that vehicle would carry, and the transitions are written conditionally.
- **§4.6, A6–A7 — Scope.** Descent and recovery paragraphs reframed conditionally and tied to barometric altitude; a three-item list ("logged, secured, shut down") collapsed.

### §5 Simulations and Performance Modeling

- **§5, V11 — Voice (#109).** The "hybrid modeling approach was used:" colon-and-"while" sentence rewritten into "two complementary models," each described in turn.
- **§5, V12 — Voice (#111).** The "Two qualitative findings" paragraph (flagged as not sounding like the author at all) restructured with topic leads ("The first concerns the timing of control authority… The second concerns recovery"). The apogee mention was dropped here to stay consistent with the rescope.
- **§5, #112 — considered, no change.** The "why this would be hard to model / weeks to rebuild" addition was left out; the airspace restriction already explains why validation was simulation-based, and over-justifying risked reading as an excuse.

### §6 Simulation-Based Validation

- **§6.3, V13 — Voice/Expand (#119).** Added the commanded-versus-actual deflection explanation (the actual angle trails the commanded because the servo can only slew so fast) and re-toned the post-burnout drift sentence, splitting its semicolon.
- **§6.5, V14 — Voice/Word (#122, #123).** Removed "aerodynamic transient" and re-toned the disturbance-rejection result while keeping the 1.3° and 0.4 s figures.
- **§6.6, V15 — Voice/Expand (#124, #125).** Matched the register, removed a duplicated commanded-versus-actual sentence, and added why low cumulative saturation matters (a controller saturated for much of the burn would have little authority left).
- **§6.8, E1 — Expand (#128).** Grounded the 0.025 N·m figure as an estimate of the aerodynamic torque a modest wind gust would impose at this scale.
- **§6.9, E2 — Expand (#129).** Described the sensitivity sweep exactly as the figures code runs it: one parameter varied at a time with the others at nominal, the baseline simulation re-run from the same 5° offset for each setting. Also removed a colon.
- **§6.11, V16 — Voice (#132).** The two flagged Monte Carlo sentences rewritten; "While individual traces diverge… the results collectively demonstrate" replaced with plainer prose.
- **§6.12, D6 — Define (#134).** "Aeroelastic coupling" defined as the feedback between airframe flexing and the aerodynamic loads that flexing creates.
- **Figure 5 caption, C1 — Voice (#126).** Tightened; dropped the em-dash title separator and the result re-argument ("confirming gains are appropriate…").
- **Figure 6 caption, C2 — Voice (#127).** Tightened; removed "it does not affect closed-loop stability" editorializing.
- **Figure 9 caption, C3 — Voice (#130).** Rewritten into descriptive prose; removed the em-dash separator and colon-labels.
- **Figure 10 caption, C4 — Voice (#133).** Minor tightening; "100/100 trials recovered" → "All 100 trials recovered."

### §7 Conclusion

- **§7, B1 — Correctness (#135).** Citation "(8, 10)" → "(8)": the sentence is about MPU-6050 sensor data only, so the Teensy citation was removed.
- **§7, V17 — Voice (#136).** The "Monte Carlo analysis further validated the robustness… under compounded uncertainty, demonstrating that failure modes were governed by" sentence rewritten in plainer terms.
- **§7, #137 — Word.** "a **meaningful** simplification" → "a **significant** simplification."
- **§7, #140 — Add (verified).** Added a citation for multi-stage thrust vectoring: BPS.Space built a three-stage actively guided rocket ("Shreeek"), confirmed by web search before citing. Worded as "has been built at the hobby scale," not "flown flawlessly," and points to the existing reference 2.

### Appendix A

- **A.2, A8–A10 — Scope (#141).** Apogee-detection appendix reframed as intended free-flight design; the Listing 2 caption relabeled "intended free-flight design, not implemented on the test-stand hardware"; the timeout paragraph made conditional.
- **A.7.2, V18 — Voice (#144).** Roll-drift paragraph simplified and the em-dashes/en-dash removed; "utilizes" → "use," "Consequently" → "therefore."
- **A.7.4, #145/#146 — Word/Add.** "These factors may introduce additional non-linearities during actual flight trials" → "For a vehicle of this size and brief flight, these effects are minor, though they could introduce additional nonlinearities in a real flight," adding the at-this-scale qualifier and fixing the spelling/wording.

---

## Items considered and intentionally left unchanged

- **#6 (trial-and-error / self-initiative clause in the Intro)** — declined. The Conclusion already lands this; repeating it in the Introduction would be redundant and tonally off.
- **#7 (how to use the material)** — declined. A journal article is not a how-to guide; the "educational and reproducible reference" framing already covers it.
- **#31 (components must work in tandem)** — declined. The §2.6 opening already states this.
- **#48 (expand the Kᵢ = 0 / PD explanation)** — declined. Between the existing paragraph and the windup definition (D4), the passage now reads completely.
- **#97 ("transition criteria")** — left as written, by author's choice.
- **#14 (10° gimbal vs 12° recoverable tilt)** — no edit needed; §3.4.2 already keeps them distinct and explains why the recoverable angle exceeds the mechanical limit.

---

## Outstanding (manuscript action still pending)

- **Gimbal diagram (#12).** A labeled CAD render of the two-axis gimbal, to be inserted in §2.2 showing the two pivot axes, the two MG90S servos and linkages, the motor mount, the vehicle longitudinal axis, and the deflection angle δ. The LaTeX `figure` block and `\ref` wiring were provided; this becomes Figure 1 and renumbers the §6 figures automatically (references use `\ref{}`, so nothing breaks).

*Submission-portal items (Scholastica metadata, keyword count, title-page details, article type) are tracked separately from this manuscript-text changelog.*
