# Draft figures — 2026-08-12 (engineering record, NOT normative)

Two presentation figures drafted 2026-08-12, kept under `other/` because each
deviates from the committed system in known ways. The code and `docs/` are
authoritative; these are drafts pending correction.

**flowchart_round1_draft.jpg** — Round 1 logic overview. Known deviations from
`src/Round 1/round 1/round 1.ino`:

1. Omits the WAIT_FOR_START state (GPIO32 start button) — the firmware boots
   into it and captures the heading reference at the button press.
2. Step 8 shows turn completion at |error| <= 90 deg; the firmware settles at
   <= 15 deg (TURN_SETTLED_DEG) — 90 deg is trivially true the moment a turn
   starts.
3. Shows an immediate stop after 12 turns; the firmware runs a settled-gated
   FINAL_RUN_MS (500 ms) run-on before braking.

**wiring_illustration_draft.jpg** — component illustration (Fritzing-style).
Known deviation: draws four TF-Luna rangefinders; the vehicle carries three
(left / centre / right). Pack capacity (2200 mAh 11.1 V) is drawn correctly.
`schemes/circuit_diagram_complete_2026-08-11.jpg` remains the wiring
reference.
