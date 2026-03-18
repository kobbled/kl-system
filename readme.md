# systemlib

Robot controller interface layer for Ka-Boost — vector constructors, date/time, PNS program selection, dual-arm leader frames, and the `t_DATA_TYPE` union that enables generic algorithms across the library stack.

---

## Overview

Karel has no built-in vector literal syntax, no polymorphic type containers, and no clean API for FANUC system variables. `systemlib` fills all three gaps. It is a Layer 1 dependency — virtually every Ka-Boost module above Layer 0 includes at least one of its headers. The most-used pieces are `VEC()`, `ZEROPOS()`, and `t_DATA_TYPE`.

You need this module when you:
- Need to construct `VECTOR` or `VECTOR2D` values inline
- Need to initialize a pose from the robot's zero configuration
- Are storing mixed types in a generic container (`hash`, `iterator`, `queue`)
- Are writing Karel that coordinates dual-arm (coordinated motion) leader frames
- Need date/time strings or PNS program-number decoding from UOPO pins

---

## Files

| File | Purpose |
|------|---------|
| `src/systemlib.kl` | All compiled routines (~240 lines) |
| `include/systemlib.klh` | Public routine declarations (extern headers for other modules) |
| `include/systemlib.klt` | Aggregator — `%include`s all type headers in one shot |
| `include/systemlib.datatypes.klt` | `t_DATA_TYPE` union struct |
| `include/systemlib.datatype.klh` | Callback function declarations for generic comparison |
| `include/systemlib.codes.klt` | Type constants (`C_INT`, `C_REAL`, …) and comparator constants (`C_GREATER`, …) |
| `include/systemlib.types.klt` | Atomic wrapper structs (`t_INTEGER`, `t_REAL`, `VECTOR2D`, etc.) |
| `include/systemlib.macros.klt` | `DEFAULT_*` and `MAX/MIN_INTEGER` constants |
| `include/systemlib.generics.klt` | GPP macro `impl_glte_body` for generic comparators |
| `include/systemvars.klt` | FANUC system variable macros (`ZEROPOS`, `ZEROARR`, `TOTAL_GROUPS`, etc.) |
| `test/test_system.kl` | KUnit tests |
| `test/test_ls_pns.ls` | TP test program for `sys_pns2sr` |

---

## API Reference

### Vector Constructors

```karel
ROUTINE VEC(x : REAL; y : REAL; z : REAL) : VECTOR FROM systemlib
ROUTINE VEC2D(x : REAL; y : REAL) : VECTOR FROM systemlib   -- z = 0
ROUTINE compare_VEC(v1 : VECTOR; v2 : VECTOR; tolerance : REAL) : BOOLEAN FROM systemlib
```

`VEC` and `VEC2D` are the most-called routines in the codebase — they replace the verbose Karel struct literal syntax. `compare_VEC` is component-wise fuzzy equality (not norm-based).

```karel
%from systemlib.klh %import VEC, VEC2D, compare_VEC

p1 := VEC(100.0, 0.0, 500.0)
p2 := VEC2D(30.5, -12.8)           -- z is automatically 0
IF compare_VEC(p1, measured, 0.05) THEN ...   -- within 0.05mm
```

---

### Date / Time

```karel
ROUTINE system__date : STRING FROM systemlib   -- 'DD-MMM-YYYY'
ROUTINE system__time : STRING FROM systemlib   -- 'HH:MM:SS'
```

> **Note:** Time has 2-second resolution (FANUC `GET_TIME` limitation). Seconds will always be even.

```karel
%from systemlib.klh %import system__date, system__time

log_msg := system__date + ' ' + system__time + ': layer complete'
```

---

### Program Name Selection

```karel
ROUTINE system__pns_to_str : STRING FROM systemlib
```

Decodes `UOPO[24..31]` binary inputs into a FANUC PNS program name string (e.g. `'PNS1003'`). Used at program start to branch on the operator-selected program number.

TP interface: `sys_pns2sr` — call from TP to write the decoded name into an SR[].

---

### TP String Bridge

```karel
ROUTINE system__set_string(str_val : STRING) : STRING FROM systemlib
```

Pass-through for TP interface `str_set`. Allows TP programs to pass a string into Karel via `CALL str_set`.

---

### Dual-Arm Leader Frames

Used for FANUC coordinated dual-robot setups (`$CD_PAIR` system variables).

```karel
ROUTINE system__set_leader_frame(cd_pair_no : INTEGER; ldr_frm_no : INTEGER; frm : XYZWPR) FROM systemlib
ROUTINE system__get_leader_frame(cd_pair_no : INTEGER; ldr_frm_no : INTEGER) : XYZWPR FROM systemlib
ROUTINE system__mask_leader_frame(cd_pair_no : INTEGER; ldr_frm_no : INTEGER; axs : INTEGER; val : REAL) FROM systemlib
```

`mask_leader_frame` updates a single axis (1=X, 2=Y, 3=Z, 4=W, 5=P, 6=R) without disturbing the rest.

TP interfaces: `sys_setldr`, `sys_getldr`, `sys_mskldr`.

```karel
-- Write full frame
system__set_leader_frame(1, 2, target_pose)

-- Update Z only
system__mask_leader_frame(1, 2, 3, 425.0)

-- Read back
current := system__get_leader_frame(1, 2)
```

---

### Utility

```karel
ROUTINE system__int_to_bool(int : INTEGER) : BOOLEAN FROM systemlib
ROUTINE system__get_tpkey_status(tpkey_no : INTEGER; flag_no : INTEGER) FROM systemlib
```

`get_tpkey_status` reads a teach-pendant key input (`TPIN[n]`) and writes the result to a flag register (`F[flag_no]`).

---

### Generic Comparison — `system__tdata_glte`

```karel
ROUTINE system__tdata_glte(data1, data2 : t_DATA_TYPE; typ : INTEGER; comparator : INTEGER) : BOOLEAN FROM systemlib
```

Polymorphic comparison for `t_DATA_TYPE` values. Used internally by `graph`, `math` (path sort), and `iterator` modules for generic sorting.

```karel
%include systemlib.datatypes.klt
%include systemlib.codes.klt

VAR a, b : t_DATA_TYPE

a.int := 5 ; b.int := 10
IF system__tdata_glte(a, b, C_INT, C_LESSER) THEN ...   -- TRUE: 5 < 10

a.vec := VEC(1,0,0) ; b.vec := VEC(3,4,0)
IF system__tdata_glte(a, b, C_VEC, C_LESSER) THEN ...   -- compares norms: 1.0 < 5.0 → TRUE
```

**Comparison behavior by type:**
- `C_INT`, `C_REAL`, `C_STR` — direct field comparison
- `C_VEC` — compares Euclidean norms (not component-wise)
- `C_POS` — 6-DOF metric: `sqrt(dx²+dy²+dz² + (dw·π/180)² + (dp·π/180)² + (dr·π/180)²)`
- Equality uses epsilon = 0.01 for all floating-point types

---

## Types

### `t_DATA_TYPE`

A union struct that lets one variable hold any Ka-Boost primitive. The backbone of all generic containers.

```karel
%include systemlib.datatypes.klt

TYPE t_DATA_TYPE FROM systemlib = STRUCTURE
    int    : INTEGER
    rl     : REAL        -- NOT "real" — Karel keyword conflict!
    str    : STRING[32]
    bool   : BOOLEAN
    vec    : VECTOR
    pos    : XYZWPR
    posext : XYZWPREXT
    cnfg   : CONFIG
ENDSTRUCTURE
```

### Atomic Wrapper Types

Karel `PATH` nodes must be structures. Use these wrappers to store atomic values in `queue` or `iterator` PATH collections:

```karel
%include systemlib.types.klt

TYPE t_INTEGER  = STRUCTURE { v : INTEGER  }
TYPE t_REAL     = STRUCTURE { v : REAL     }
TYPE t_STRING16 = STRUCTURE { v : STRING[16] }
TYPE t_BOOL     = STRUCTURE { v : BOOLEAN  }
TYPE t_VECTOR   = STRUCTURE { v : VECTOR   }
TYPE t_POSE     = STRUCTURE { v : XYZWPR   }
TYPE VECTOR2D   = STRUCTURE { x : REAL; y : REAL }
TYPE VECTOR2Di  = STRUCTURE { x : INTEGER; y : INTEGER }
```

---

## System Variable Macros (`systemvars.klt`)

| Macro | Expands to | Use |
|-------|-----------|-----|
| `ZEROPOS(g)` | `$MOR_GRP[g].$NILPOS` | Zero XYZWPR for group g — extract `.config_data` |
| `ZEROARR` | `$MOR_GRP[1].$CUR_NOM_ANG` | Current nominal joint angles of group 1 |
| `DYNAMIC_LEADER(f,l)` | `$CD_PAIR[f].$LEADER_FRM[l]` | Direct macro access to coordinated leader frame |
| `TOTAL_GROUPS` | `ARRAY_LEN($GROUP)` | Number of robot groups |
| `GROUP_KINEMATICS(g)` | `$SCR_GRP[g].$KINEM_ENB` | Kinematics enabled flag for group g |
| `CURRENT_UTOOL` | `$MNUTOOLNUM[1]` | Active tool frame number |
| `CURRENT_UFRAME` | `$MNUFRAMENUM[1]` | Active user frame number |

---

## Type and Comparator Codes

Include `systemlib.codes.klt` to get:

```
C_INT=1  C_REAL=2  C_STR=3  C_BOOL=4  C_VEC=5  C_POS=6  C_POSEXT=7  C_CONFIG=8
C_REAL_ARR=9  C_INT_ARR=10  C_STR_ARR=11  C_VEC2D=12

C_GREATER=1  C_LESSER=2  C_EQUAL=3  C_GREATEREQL=4  C_LESSEREQL=5
```

---

## Common Patterns

### Extracting CONFIG from the zero pose

Karel's `POS()` constructor requires a `CONFIG`. The robot's zero pose provides one without hard-coding:

```karel
%include systemvars.klt

-- Build a pose using the robot's default config
my_pose := POS(100.0, -50.0, 400.0, 0.0, 90.0, 0.0, (ZEROPOS(1).config_data))

-- Initialize a frame variable to zero
ref_frame := ZEROPOS(1)
```

---

### Storing mixed types in a generic container

```karel
%include systemlib.datatypes.klt
%include systemlib.codes.klt

-- In a hash or iterator, the node value is t_DATA_TYPE:
VAR entry : t_DATA_TYPE

entry.rl := 3.14          -- store a real (field is "rl", not "real")
entry.vec := VEC(1,2,3)   -- or a vector

-- The container (hash, graph, iterator) carries t_DATA_TYPE nodes
-- and system__tdata_glte sorts/compares them generically
```

---

### Wrapping atomic types for PATH collections

The `iterator` and `queue` modules use PATH nodes, which must be structs:

```karel
%include systemlib.types.klt

-- In queue.klc — wrap an integer value into a PATH-compatible struct:
VAR node : t_INTEGER
node.v := my_int_value
APPEND_NODE(my_path)
my_path[PATH_LEN(my_path)] := node

-- Retrieve:
result_int := my_path[1].v
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| `nde.real` on `t_DATA_TYPE` | Compile error — `REAL` is a Karel keyword | Use `nde.rl` (the field is deliberately named `rl`) |
| Using `ZEROPOS(g)` as a motion target | Robot drives to literal zero position | Use only for config extraction: `ZEROPOS(1).config_data` |
| Expecting per-component vector compare from `system__tdata_glte` with `C_VEC` | Wrong result — only norm is compared | Implement component-wise logic separately; `tdata_glte` gives `‖v‖` comparison only |
| Omitting `%include systemlib.datatypes.klt` before `VAR : t_DATA_TYPE` | TYPE not found / compile error | Always include the type header before any variable declaration |
| Expecting sub-second precision from `system__time` | Seconds are always even (0, 2, 4, …) | Karel `GET_TIME` has 2-second resolution — this is a hardware limitation |
| Passing `POSITION` (homogeneous 4×4) to `set_leader_frame` | Type mismatch | The routine takes `XYZWPR`; convert with `pose__mat_to_pose` if needed |

---

## Build Flow

`systemlib` compiles as a single `.pc` binary from `src/systemlib.kl`. All headers in `include/` are preprocessor-only (types, macros, constants) — they generate no compiled output.

rossum places `include/` on the GPP search path so downstream modules can `%include systemlib.datatypes.klt` without specifying a path.

Five TP-callable wrappers are declared in `package.json` under `tp-interfaces`. They are compiled alongside the main source and deployed as separate TP programs (`sys_pns2sr.pc`, `sys_setldr.pc`, etc.).

See the [Ka-Boost root readme](../../readme.md) for full build and deployment instructions.
