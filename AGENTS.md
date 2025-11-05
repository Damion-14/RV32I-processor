# Repository Guidelines

## Project Structure & Module Organization
The Verilog source lives in `rtl/`; keep every synthesizable module (e.g. `hart.v`, `alu.v`, `control.v`) flat in that directory. The `tb/` folder holds the primary testbench (`tb.v`) plus the `program.mem` image consumed at runtime. Assembly tests reside in `tests/asm/`, with Docker-backed tooling to emit fresh memory images. Generated traces and waveforms belong in `traces/`. Legacy synthesis drops (under `proj4_*`) are reference-only—do not overwrite them; place new implementation artifacts under a dated subdirectory.

## Build, Test, and Development Commands
- `cd tests/asm && docker pull coderkalyan/ece552-tools:latest` (fetch toolchain once per machine).
- `cd tests/asm && make PROGRAM=branch_test` assembles `branch_test.asm`, producing `tb/program.mem`.
- `iverilog -g2012 -o tb/a.out tb/tb.v rtl/*.v` compiles the design and testbench into a runnable simulation image.
- `vvp tb/a.out` executes the simulation, writing execution traces to `traces/`.
- `iverilog -Wall -g2012 -o /tmp/lint rtl/*.v` is a quick syntax/lint sanity check before commits.

## Coding Style & Naming Conventions
Write Verilog-2001/2012 with four-space indentation and align port lists vertically for readability. Use `lower_snake_case` for signals and nets (`o_retire_valid`), `ALL_CAPS` for parameters/constants (`RESET_ADDR`), and module names in `lower_snake_case` matching filenames. Maintain the existing header comment blocks and keep `default_nettype none` at file tops. Prefer explicit bit widths and blocking/non-blocking assignments that mirror the shipped modules.

## Testing Guidelines
Add new assembly tests in `tests/asm/` named with a two-digit prefix plus intent (e.g. `05muldiv.asm`). Regenerate `program.mem` via `make PROGRAM=...` and check the diff into version control when relevant. Run `vvp tb/a.out` until the retire interface halts on `ebreak` and inspect `traces/` output for mismatches. When modifying control flow or memory, capture before/after register dumps in the PR description. Target full RV32I coverage; if a scenario cannot be automated, document the manual check you performed.

## Commit & Pull Request Guidelines
Follow the existing log style: a short present-tense summary (`update decoder`, `fix hazard stall`). Squash noisy intermediate commits locally. Each PR should link the tracked issue (if any), summarize architectural impacts, list the tests run (command + result), and attach waveforms or trace snippets when behaviour changes. Tag reviewers on shared modules and request synthesis sign-off before merging timing-sensitive updates.
