# tdc_gpx_top operation and code review deck plan

Audience: FPGA/firmware developers and system integrators who need to operate, debug, and extend the current `tdc_gpx_top` module and its submodules.

Objective: Explain the operating concept of the top-level TDC-GPX controller, fill missing operational assumptions for software/supervisor usage, and report code-quality or maintainability risks found during source review.

Narrative arc:
1. Define the source scope and top-level architecture.
2. Explain the three clock/control domains and CDC safety contract.
3. Walk through the normal measurement lifecycle from CSR configuration through TDC drain, decode, cell build, face assembly, header insertion, and AXI-Stream output.
4. Describe recovery, observability, and status interpretation.
5. Summarize review findings, operational guardrails, and next actions.

Slide list:
1. Cover and executive intent
2. Source scope and review lens
3. Top-level module map
4. Clock and CDC contract
5. CSR and command plane
6. Normal run lifecycle
7. Cluster 1 chip-control path
8. TDC bus timing model
9. Decode and raw-event path
10. Cell builder and slope split
11. Output stage and VDMA packet
12. Face sequencer and config snapshot policy
13. Error recovery and fault propagation
14. Status and observability surface
15. Backpressure and timeout protections
16. Code review findings
17. Operational gaps to close
18. Verification and test coverage
19. Recommended next actions

Source plan:
- Primary VHDL files: `tdc_gpx_top.vhd`, `tdc_gpx_config_ctrl.vhd`, `tdc_gpx_face_seq.vhd`, `tdc_gpx_chip_ctrl.vhd`, `tdc_gpx_chip_run.vhd`, `tdc_gpx_bus_phy.vhd`, `tdc_gpx_decode_pipe.vhd`, `tdc_gpx_cell_pipe.vhd`, `tdc_gpx_cell_builder.vhd`, `tdc_gpx_face_assembler.vhd`, `tdc_gpx_header_inserter.vhd`, `tdc_gpx_status_agg.vhd`, `tdc_gpx_csr_pipeline.vhd`, `tdc_gpx_csr_chip.vhd`, `tdc_gpx_pkg.vhd`, and `tdc_gpx_cfg_pkg.vhd`.
- Support docs: `Doc/register_map.md`, `Doc/known_issues.md`, `Doc/status_observability.md`.
- Local test references: `tb_tdc_gpx_*.vhd`.

Visual system:
- Editable PowerPoint shapes and text only for block diagrams, flow diagrams, checklists, and findings.
- Palette: light technical canvas with navy, teal, amber, green, and red accents.
- Typography: Malgun Gothic for Korean text, Aptos Mono for code labels.

Editability plan:
- All titles, body copy, diagrams, tables, and callouts are authored as editable PowerPoint objects.
- Preview rendering and text-inspection records are generated as scratch verification artifacts.
