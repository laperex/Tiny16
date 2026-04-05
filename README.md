# Tiny16

A **parameterized 8/16-bit CISC accumulator-based processor** implemented in a single SystemVerilog file, complete with ALU, hardwired control unit, memory-mapped PC, and a built-in Fibonacci program burned into RAM at reset.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
  - [Block Diagram](#block-diagram)
  - [Modules](#modules)
  - [The ALU — Universal Logic Function Generator](#the-alu--universal-logic-function-generator)
  - [The Memory-Mapped Program Counter](#the-memory-mapped-program-counter)
  - [The Control Unit — Hardwired Microstep Sequencer](#the-control-unit--hardwired-microstep-sequencer)
  - [Processor Status Register (PSR)](#processor-status-register-psr)
- [Instruction Set Architecture](#instruction-set-architecture)
  - [Instruction Encoding](#instruction-encoding)
  - [Addressing Modes](#addressing-modes)
  - [Instruction Reference](#instruction-reference)
- [Control Signals](#control-signals)
- [Execution Microsteps](#execution-microsteps)
- [Built-in Fibonacci Program](#built-in-fibonacci-program)
- [Simulating](#simulating)
- [Halt Mechanism](#halt-mechanism)

---

## Overview

Tiny16 is a Von Neumann, accumulator-based processor where a single register (`A`) is the implicit source and destination of all arithmetic and logic operations. The design is width-parameterized — it defaults to 16 bits but the testbench instantiates it at 8 bits (`processor #(8)`).

Key design highlights:

- **No dedicated PC register** — the program counter lives at RAM address `0`, making it software-readable and writable.
- **Universal ALU** — a per-bit 4-entry lookup table (`L[3:0]`) makes the ALU a configurable logic function generator for any two-variable Boolean function, combined with a carry-ripple adder.
- **Hardwired control unit** — no microcode ROM; a 3-bit step counter indexes a combinational `decode` expression.
- **Auto-terminating microcycle** — when `decode == 0`, the step counter resets, giving variable-length instruction execution.
- **Single source file** — the entire CPU is `tiny.sv`.

---

## Architecture

### Block Diagram

```
                    ┌──────────────────────────────────────────────────────┐
                    │                      PROCESSOR                       │
                    │                                                      │
clk, reset ────────►│                                                      │
                    │  ┌────────────────────────────────────────────────┐  │
                    │  │             DATA BUS (WIDTH bits)              │  │
                    │  └──┬────────────────┬───────────────┬────────────┘  │
                    │     │                │               │               │
                    │  ┌──▼──┐          ┌──▼──┐         ┌──▼─────────┐     │
                    │  │ ALU │          │ MUX │         │   MEMORY   │     │
                    │  │     │◄─────────│ 2x1 │◄────────│  (RAM +    │     │
                    │  │ L,  │          │     │  AI     │   MAR)     │     │
                    │  │ Cn, │          │     │  AS     │            │     │
                    │  │ LS  │          └──▲──┘         │ MI: MAR←   │     │
                    │  │     │             │            │ RI: RAM←   │     │
                    │  │  E  │         ACCUMULATOR      │ read → bus │     │
                    │  └──┬──┘         (sync reg)       └──────┬─────┘     │
                    │     │                                    │           │
                    │  PSR│                            ┌───────▼──────┐    │
                    │  ┌──▼──┐              ┌─────────►│  INSTR REG   │    │
                    │  │ PSR │              │   II     │  (inst[7:0]) │    │
                    │  │[3:0]│              │          └───────┬──────┘    │
                    │  └──┬──┘              │                  │           │
                    │     │         ┌───────▼──────────────────▼───────┐   │
                    │     └────────►│          CONTROL UNIT            │   │
                    │               │  (step counter + decode logic)   │   │
                    │               │  AI AS PI PS MI RI ES Cn LS II   │   │
                    │               │  L[3:0]                          │   │
                    │               └──────────────────────────────────┘   │
                    └──────────────────────────────────────────────────────┘
```

### Modules

| Module | Description |
|---|---|
| `processor` | Top-level: wires all submodules together |
| `alu16` | Parameterized ALU with per-bit LUT and carry-ripple adder |
| `sync_mux_2x1` | Synchronous 2:1 mux — implements the accumulator register |
| `memory` | Synchronous RAM with MAR, and a Fibonacci program loaded at reset |
| `controlunit` | Hardwired microstep sequencer, produces all control signals |
| `instructionregister` | Latches instruction byte, asserts `halt` on `0xFF` |
| `testbench` | Simulation driver: clock, reset, VCD dump, 2000-tick timeout |

---

### The ALU — Universal Logic Function Generator

The ALU (`alu16`) is built around a per-bit **4-entry lookup table** driven by `L[3:0]`:

```verilog
// For each bit i:
lB[i] = A[i] ? (B[i] ? L[3] : L[2])
              : (B[i] ? L[1] : L[0])
```

This makes `L[3:0]` the truth table of **any** two-input Boolean function of `A[i]` and `B[i]`. The result `lB` is then fed into a carry-ripple adder:

```
lA  = LS ? 0 : A          // LS zeroes the accumulator input
tE  = {0, lA} + {0, lB} + Cn
E   = tE[WIDTH-1:0]
```

The control unit combines `L[3:0]`, `LS`, and `Cn` to produce all arithmetic and logic modes:

| Alias | L[3:0] | LS | Cn | Operation | Result |
|---|---|---|---|---|---|
| `GEN_0` | `0000` | 1 | 0 | `0 + 0` | Constant 0 |
| `GEN_1` | `1111` | 1 | 0 | `0 + ~0` | Constant all-ones |
| `BUF_B` | `1010` | 0 | 0 | `A + B` | Addition |
| `BUF_B + Cn` | `1010` | 0 | 1 | `A + B + 1` | Add with carry |
| `BUF_B + LS` | `1010` | 1 | 0 | `0 + B` | Pass B through (lA=0) |
| `BUF_A` | `1100` | 0 | 0 | `A + A` | Route A to write bus |
| `NEG_B` | `0101` | 0 | 0 | `A + ~B` | Subtract with borrow |
| `NEG_B + Cn` | `0101` | 0 | 1 | `A + ~B + 1` | Two's complement subtract |

**Why `L=1100` routes A:** with L[3]=1, L[2]=1: `lB[i] = A[i] ? 1 : 0 = A[i]`. So `lB = A` regardless of B. Combined with `LS=0`: output = `A + A`. Combined with `LS=1`: output = `0 + A = A`. The control unit uses this with `MI|RI` to write A directly to the write bus.

---

### The Memory-Mapped Program Counter

There is **no dedicated PC register**. Instead, **RAM address `0` holds the current program counter value**, initialized at reset:

```verilog
ram[0] = org - 1;   // org = 100, so ram[0] = 99
```

It is pre-decremented because the fetch cycle increments it before use. The two fetch steps (0→1 and 2→3) each read RAM[0], add 1 via `BUF_B | Cn | LS`, write the result back to RAM[0] (via `RI`), and simultaneously load the incremented value into the MAR (via `MI`). This advances the PC and points MAR at the next word in one step.

Consequences:

- The PC is **readable and writable** by any `LOADA`/`STORE` instruction targeting address 0.
- `JUMP` is implemented by loading the operand into A, then writing A back to RAM[0] — overwriting the PC.
- There is no return address mechanism or call stack.

---

### The Control Unit — Hardwired Microstep Sequencer

The control unit maintains a **3-bit step counter** advancing on the negative clock edge. A large combinational expression maps `(counter, TYPE_*, opcode, psr)` → a 16-bit `decode` word containing every control signal.

```verilog
always @(negedge clk or posedge reset or posedge (decode == 0)) begin
    if (reset == 1 || decode == 0)
        counter <= 0;
    else
        counter <= counter + 1;
end
```

When `decode == 0` — meaning no control line needs to be active for the current step — the counter resets to 0, ending the instruction cycle. This gives the processor **variable-length execution** without a microcode ROM or fixed pipeline depth.

---

### Processor Status Register (PSR)

The 4-bit PSR is latched from the ALU result when the `PI` control line is asserted:

| Bit | Flag | Condition |
|---|---|---|
| `psr[0]` | **Carry (C)** | `tE[WIDTH]` — carry-out of the adder |
| `psr[1]` | **Negative (N)** | `E[WIDTH-1]` — MSB of result is 1 |
| `psr[2]` | **Zero (Z)** | `E == 0` |
| `psr[3]` | **Overflow (V)** | Always 0 (reserved; logic stub present but commented out) |

Conditional branches (`JUMPZ`, `JUMPC`, `JUMPN`) read the appropriate PSR bit directly in the control unit's decode expression, requiring no separate comparison instruction.

---

## Instruction Set Architecture

### Instruction Encoding

Instructions are **8 bits wide** and are always followed by exactly one operand word, making all instructions two memory words long.

```
 7   6   5   4   3   2   1   0
┌───┬───┬───────────────────────┐
│ M1│ M0│     OPCODE [5:0]      │
└───┴───┴───────────────────────┘
```

### Addressing Modes

| M1 | M0 | Mode | Operand meaning |
|---|---|---|---|
| 0 | 0 | **IMM** — Immediate | Next word is the literal value |
| 0 | 1 | **ABS** — Absolute | Next word is the memory address of the value |
| 1 | 0 | **OFF** — Offset | Defined, not yet implemented |
| 1 | 1 | **REG** — Register | Defined, not yet implemented |

### Instruction Reference

| Mnemonic | Opcode[5:0] | Mode | Operation | Flags |
|---|---|---|---|---|
| `LOADI imm` | `000001` | IMM | `A ← imm` | — |
| `LOADA addr` | `000001` | ABS | `A ← MEM[addr]` | — |
| `STORE addr` | `000010` | ABS | `MEM[addr] ← A` | — |
| `JUMP addr` | `000100` | ABS | `PC ← addr` | — |
| `JUMPZ addr` | `000101` | ABS | `if Z: PC ← addr` | — |
| `JUMPC addr` | `000110` | ABS | `if C: PC ← addr` | — |
| `JUMPN addr` | `000111` | ABS | `if N: PC ← addr` | — |
| `ADD` | `100000` | IMM/ABS | `A ← A + operand` | C, N, Z |
| `ADC` | `100001` | IMM/ABS | `A ← A + operand + C` | C, N, Z |
| `SBB` | `100010` | IMM/ABS | `A ← A − operand − borrow` | C, N, Z |
| `SUB` | `100011` | IMM/ABS | `A ← A − operand` | C, N, Z |
| `NAND`, `AND` | `010100–010101` | — | Defined, not yet decoded | — |
| `NOR`, `OR` | `010110–010111` | — | Defined, not yet decoded | — |
| `XNOR`, `XOR` | `011000–011001` | — | Defined, not yet decoded | — |

> Logic instructions have opcodes reserved in both the `memory` and `controlunit` modules but are not yet wired into the microstep decode table.

---

## Control Signals

The 16-bit `decode` word maps to individual control lines:

| Bits | Signal | Function |
|---|---|---|
| 0 | `AI` | **Accumulator In** — enable write to accumulator register |
| 1 | `AS` | **Accumulator Select** — `0`: use read bus; `1`: use write bus as accumulator input |
| 2 | `PI` | **PSR In** — latch ALU flags into PSR |
| 3 | `PS` | **PSR Select** — choose PSR source (reserved) |
| 4 | `MI` | **MAR In** — load Memory Address Register from write bus |
| 5 | `RI` | **RAM In** — write `write_bus` into `RAM[MAR]` |
| 6 | `ES` | **Enable PSR source** — route PSR onto write bus instead of ALU output |
| 7 | `Cn` | **Carry In** — add 1 to ALU result |
| 8 | `LS` | **Load Select** — zero the A input to ALU (enables constant/B-passthrough generation) |
| 9 | `II` | **Instruction In** — latch read bus into instruction register |
| 15:11 | `L[4:0]` | ALU LUT select (`L[3:0]` used; bit 4 unused) |

---

## Execution Microsteps

Every instruction shares the same **4-step fetch preamble** (steps 0–3), then executes in up to 2 additional steps (4–5). The cycle terminates early whenever `decode == 0`.

### Fetch — All Instructions (Steps 0–3)

| Step | Signals Active | Action |
|---|---|---|
| 0 | `GEN_0 \| MI` | ALU output = 0; MAR ← 0 (point to PC location) |
| 1 | `BUF_B \| Cn \| MI \| RI \| LS` | ALU: `0 + RAM[0] + 1`; RAM[0] ← PC+1; MAR ← PC+1 (point to instruction byte) |
| 2 | `II \| GEN_0 \| MI` | inst ← RAM[MAR]; MAR ← 0 (point to PC again) |
| 3 | `BUF_B \| Cn \| MI \| RI \| LS` | ALU: `0 + RAM[0] + 1`; RAM[0] ← PC+2; MAR ← PC+2 (point to operand) |

After step 3: `inst` holds the opcode, MAR points to the operand word.

### Execute — IMM Instructions (Step 4 Only)

| Instruction | Step 4 Signals | Action |
|---|---|---|
| `LOADI` | `AI` | A ← RAM[MAR] |
| `ADD imm` | `BUF_B \| PI \| AS \| AI` | A ← A + imm; latch flags |
| `ADC imm` | `BUF_B \| PI \| AS \| AI \| Cn` | A ← A + imm + C; latch flags |
| `SBB imm` | `NEG_B \| PI \| AS \| AI` | A ← A + ~imm (subtract with borrow); latch flags |
| `SUB imm` | `NEG_B \| PI \| AS \| AI \| Cn` | A ← A + ~imm + 1 = A − imm; latch flags |

### Execute — ABS Instructions (Steps 4–5)

| Instruction | Step 4 | Step 5 |
|---|---|---|
| `LOADA addr` | `BUF_B \| MI \| LS` → MAR ← operand (the address) | `AI` → A ← RAM[MAR] |
| `STORE addr` | `BUF_B \| MI \| LS` → MAR ← operand | `BUF_A \| RI \| LS` → RAM[MAR] ← A |
| `JUMP addr` | `GEN_0 \| MI \| AI` → A ← operand; MAR ← 0 | `BUF_A \| MI \| RI \| LS` → RAM[0] ← A (new PC) |
| `JUMPZ/C/N` (taken) | Same as JUMP step 4 | Same as JUMP step 5 |
| `JUMPZ/C/N` (not taken) | `decode == 0` → cycle ends immediately | — |
| `ADD addr` | `BUF_B \| MI \| LS` → MAR ← addr | `BUF_B \| PI \| AS \| AI` → A ← A + MEM[addr]; flags |
| `ADC addr` | `BUF_B \| MI \| LS` → MAR ← addr | `BUF_B \| PI \| AS \| AI \| Cn` → A ← A + MEM[addr] + C; flags |
| `SBB addr` | `BUF_B \| MI \| LS` → MAR ← addr | `NEG_B \| PI \| AS \| AI` → A ← A − MEM[addr] − borrow; flags |
| `SUB addr` | `BUF_B \| MI \| LS` → MAR ← addr | `NEG_B \| PI \| AS \| AI \| Cn` → A ← A − MEM[addr]; flags |

---

## Built-in Fibonacci Program

At reset, the `memory` module loads a Fibonacci computation program directly into RAM. It computes the n-th Fibonacci term iteratively using a loop counter and two running values.

### Memory Layout

| Address | Label | Initial Value | Role |
|---|---|---|---|
| `0` | `PC` | `99` | Program counter (pre-decremented; fetch adds 1) |
| `10` | `tmp` | `0` | Temporary swap variable |
| `11` | `first` | `0` | F(n) |
| `12` | `second` | `1` | F(n+1) |
| `13` | `count` | `7` | Loop iteration counter |
| `100–199` | code | — | Program instructions |
| `200` | finish | `0xFF` | Halt sentinel |

### Program Listing

```asm
; Computes the 7th Fibonacci number, result in MEM[11]

100: LOADA  count       ; A = count
102: SBB    0           ; A = count - 0 - borrow  (sets Z when count reaches 0)
104: STORE  count       ; count = A  (decrement in place)
106: JUMPZ  200         ; if count == 0, halt
108: LOADA  second      ; A = F(n+1)
110: STORE  tmp         ; save F(n+1) temporarily
112: ADDA   first       ; A = F(n+1) + F(n)
114: STORE  second      ; second = F(n) + F(n+1)   <- new F(n+1)
116: LOADA  tmp         ; A = old F(n+1)
118: STORE  first       ; first = old F(n+1)        <- new F(n)
120: JUMP   100         ; loop

200: 0xFF               ; HALT
```

After 7 iterations the result (F(7) = 13 for seed first=0, second=1) is left in `RAM[11]`. Modify `ram[count]` at reset to compute a different term.

---

## Simulating

The `testbench` module instantiates an 8-bit processor and drives the simulation:

```bash
# With Icarus Verilog
iverilog -g2012 -o tiny tiny.sv
vvp tiny

# Inspect waveforms
gtkwave testbench.vcd
```

The simulation terminates when `halt` is asserted or after a 2000-tick timeout, whichever comes first. The `$monitor` statement traces `clk`, `reset`, and `halt` at every change.

---

## Halt Mechanism

The instruction register continuously checks its own output:

```verilog
if (inst == 2**WIDTH - 1)   // 0xFF for 8-bit, 0xFFFF for 16-bit
    halt <= 1;
```

`halt` propagates to the testbench which calls `$finish`. The halt opcode (`0xFF`) is burned into RAM at address 200 by the reset sequence. In a physical implementation, `halt` would gate the clock or assert a system stop signal.
