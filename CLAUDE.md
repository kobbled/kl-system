# CLAUDE.md — lib/system (systemlib)

## Purpose

`systemlib` is a Layer 1 foundational module that provides the robot controller interface layer for Ka-Boost. It solves three distinct problems Karel leaves unaddressed: (1) no built-in vector constructors, (2) no polymorphic/generic type container, and (3) no clean API for FANUC system variables like date/time, PNS program selection, and coordinated dual-arm leader frames. Nearly every other module in Ka-Boost depends on it directly via `%include systemlib.datatypes.klt` or `%import VEC`.

---

## Repository Layout

```
lib/system/
├── package.json                     rossum manifest (name=systemlib, depends=[Strings, ktransw-macros])
├── src/
│   └── systemlib.kl                 Full implementation (~240 lines), all routines defined here
├── include/
│   ├── systemlib.klt                Aggregator — %includes all type headers
│   ├── systemlib.klh                Public routine declarations (EXTERN declarations)
│   ├── systemlib.codes.klt          Type codes C_INT/C_REAL/C_STR/... and comparator codes
│   ├── systemlib.datatypes.klt      t_DATA_TYPE union struct definition
│   ├── systemlib.datatype.klh       Callback function declarations for t_DATA_TYPE comparisons
│   ├── systemlib.generics.klt       GPP macro impl_glte_body / impl_glte_func
│   ├── systemlib.macros.klt         DEFAULT_BYTE/SHORT/REAL, MAX/MIN_INTEGER constants
│   ├── systemlib.types.klt          Atomic wrapper structs: t_INTEGER, t_REAL, t_STRING16, t_BOOL, t_VECTOR, t_POSE, VECTOR2D, VECTOR2Di
│   └── systemvars.klt               FANUC system variable macros (ZEROPOS, ZEROARR, TOTAL_GROUPS, etc.)
└── test/
    ├── test_system.kl               KUnit tests (date, time, VEC, VEC2D, compare_VEC, pns_to_str)
    └── test_ls_pns.ls               TP program exercising sys_pns2sr TP interface
```

---

## Full API Reference

### Vector Constructors (systemlib.kl)

```karel
ROUTINE VEC(x : REAL; y : REAL; z : REAL) : VECTOR FROM systemlib
```
Constructs a VECTOR with given x/y/z. Used everywhere geometric values are built inline.

```karel
ROUTINE VEC2D(x : REAL; y : REAL) : VECTOR FROM systemlib
```
Constructs a VECTOR with given x/y and z=0. Semantic helper for 2D polygon operations in `draw`.

```karel
ROUTINE compare_VEC(v1 : VECTOR; v2 : VECTOR; tolerance : REAL) : BOOLEAN FROM systemlib
```
Returns TRUE if all three components differ by less than `tolerance`. Used for point-coincidence checks.

---

### Date / Time (systemlib.kl)

```karel
ROUTINE system__date : STRING FROM systemlib
```
Returns 'DD-MMM-YYYY' (e.g. `'17-MAR-2026'`). Calls `GET_TIME` → `CNV_TIME_STR` → 9-char slice + `rstrip`.

```karel
ROUTINE system__time : STRING FROM systemlib
```
Returns 'HH:MM:SS'. Seconds are computed manually from the 5-bit field (2-second resolution → values 0–29 mapped to 0–58).

---

### Program Selection (systemlib.kl)

```karel
ROUTINE system__pns_to_str : STRING FROM systemlib
```
Reads `UOPO[24..31]` (binary select input pins), assembles an 8-bit string, converts via `bin_to_i`, adds `$SHELL_CFG.$JOB_BASE` (1000), and prepends `$PNS_PROGRAM`. Returns e.g. `'PNS1001'`. TP interface: `sys_pns2sr`.

---

### TP String Bridge (systemlib.kl)

```karel
ROUTINE system__set_string(str_val : STRING) : STRING FROM systemlib
```
Pass-through (returns input unchanged). Exists only as a TP interface (`str_set`) so TP programs can write a Karel string via `CALL str_set`.

---

### Dual-Arm Leader Frames (systemlib.kl)

```karel
ROUTINE system__set_leader_frame(cd_pair_no : INTEGER; ldr_frm_no : INTEGER; frm : XYZWPR) FROM systemlib
```
Writes `frm` into `$CD_PAIR[cd_pair_no].$LEADER_FRM[ldr_frm_no]` via `SET_VAR`. TP interface: `sys_setldr`.

```karel
ROUTINE system__get_leader_frame(cd_pair_no : INTEGER; ldr_frm_no : INTEGER) : XYZWPR FROM systemlib
```
Reads the leader frame via `GET_VAR`, returns XYZWPR. TP interface: `sys_getldr`.

```karel
ROUTINE system__mask_leader_frame(cd_pair_no : INTEGER; ldr_frm_no : INTEGER; axs : INTEGER; val : REAL) FROM systemlib
```
Updates a single axis of the leader frame without touching others. `axs`: 1=X, 2=Y, 3=Z, 4=W, 5=P, 6=R. TP interface: `sys_mskldr`.

---

### Type Conversion / Utility (systemlib.kl)

```karel
ROUTINE system__int_to_bool(int : INTEGER) : BOOLEAN FROM systemlib
```
Returns `FALSE` if `int == 0`, else `TRUE`.

```karel
ROUTINE system__get_tpkey_status(tpkey_no : INTEGER; flag_no : INTEGER) FROM systemlib
```
Reads `TPIN[tpkey_no]` and writes result into `F[flag_no]` via `registers__set_io`.

---

### Generic Comparison (systemlib.kl + systemlib.datatype.klh)

```karel
ROUTINE system__tdata_glte(data1, data2 : t_DATA_TYPE; typ : INTEGER; comparator : INTEGER) : BOOLEAN FROM systemlib
```
Polymorphic comparator dispatcher. `typ` selects which field of `t_DATA_TYPE` to compare. `comparator` selects the operator. For REAL/VEC/POS, uses epsilon = 0.01 for equality. For VEC, compares Euclidean norms. For POSE (C_POS), computes 6-DOF distance with angles converted to radians.

**Callback functions** (declared in `systemlib.datatype.klh`):
```karel
ROUTINE system__tdata_callback_int(nde : t_DATA_TYPE) : INTEGER
ROUTINE system__tdata_callback_real(nde : t_DATA_TYPE) : REAL
ROUTINE system__tdata_callback_string(nde : t_DATA_TYPE) : STRING
ROUTINE system__tdata_callback_vec(nde : t_DATA_TYPE) : REAL     -- returns ||v||
ROUTINE system__tdata_callback_pose(nde : t_DATA_TYPE) : REAL    -- returns 6-DOF distance metric
```

---

### Types

#### `t_DATA_TYPE` (systemlib.datatypes.klt)

Union struct that carries one of several data types:

```karel
TYPE t_DATA_TYPE FROM systemlib = STRUCTURE
    int  : INTEGER
    rl   : REAL              -- NOTE: field is "rl", not "real" (Karel keyword conflict)
    str  : STRING[32]
    bool : BOOLEAN
    vec  : VECTOR
    pos  : XYZWPR
    posext : XYZWPREXT
    cnfg : CONFIG
ENDSTRUCTURE
```

**Used by:** `csv` (typed cell parsing), `graph` / `iterator` (generic node data), `hash` (generic value type).

#### Atomic Wrapper Types (systemlib.types.klt)

```karel
TYPE VECTOR2D   = STRUCTURE { x : REAL; y : REAL }
TYPE VECTOR2Di  = STRUCTURE { x : INTEGER; y : INTEGER }
TYPE t_INTEGER  = STRUCTURE { v : INTEGER }
TYPE t_REAL     = STRUCTURE { v : REAL }
TYPE t_STRING16 = STRUCTURE { v : STRING[16] }
TYPE t_BOOL     = STRUCTURE { v : BOOLEAN }
TYPE t_VECTOR   = STRUCTURE { v : VECTOR }
TYPE t_POSE     = STRUCTURE { v : XYZWPR }
```

Karel `PATH` nodes must be STRUCTUREs. These wrappers allow atomic values to be stored in PATH-backed containers (`queue`, `iterator`). The `wrap`/`unwrap` pattern in `iterator` uses these types.

---

### Constants

#### Type Codes (systemlib.codes.klt)
| Constant | Value | Meaning |
|----------|-------|---------|
| `C_INT` | 1 | Integer |
| `C_REAL` | 2 | Real |
| `C_STR` | 3 | String |
| `C_BOOL` | 4 | Boolean |
| `C_VEC` | 5 | VECTOR |
| `C_POS` | 6 | XYZWPR |
| `C_POSEXT` | 7 | XYZWPREXT |
| `C_CONFIG` | 8 | CONFIG |
| `C_REAL_ARR` | 9 | REAL array |
| `C_INT_ARR` | 10 | INTEGER array |
| `C_STR_ARR` | 11 | STRING array |
| `C_VEC2D` | 12 | VECTOR2D |

#### Comparator Codes (systemlib.codes.klt)
| Constant | Value |
|----------|-------|
| `C_GREATER` | 1 |
| `C_LESSER` | 2 |
| `C_EQUAL` | 3 |
| `C_GREATEREQL` | 4 |
| `C_LESSEREQL` | 5 |

#### Default/Limit Values (systemlib.macros.klt)
```
DEFAULT_BYTE   = 255
DEFAULT_SHORT  = 32766
DEFAULT_REAL   = 1.E9
MAX_INTEGER    = 2147483646
MIN_INTEGER    = -2147483648
```

---

### System Variable Macros (systemvars.klt)

```karel
ZEROPOS(g)             -- $MOR_GRP[g].$NILPOS  — zero XYZWPR for group g
ZEROARR                -- $MOR_GRP[1].$CUR_NOM_ANG  — nominal joint angles of group 1
DYNAMIC_LEADER(f,l)    -- $CD_PAIR[f].$LEADER_FRM[l]  — direct macro access to leader frame
TOTAL_GROUPS           -- ARRAY_LEN($GROUP)
GROUP_KINEMATICS(g)    -- $SCR_GRP[g].$KINEM_ENB
CURRENT_UTOOL          -- $MNUTOOLNUM[1]
CURRENT_UFRAME         -- $MNUFRAMENUM[1]
```

---

### GPP Generics Macro (systemlib.generics.klt)

```
impl_glte_body(varname1, varname2, comparator_var, callback_func, epsilon_value)
```
Generates the SELECT/IF body for a generic ≥/≤/= comparison using a callback that extracts a comparable scalar. Used inside `system__tdata_glte`.

```
impl_glte_func(structtype, callback_func, epsilon_value)
```
Wraps `impl_glte_body` into a standalone routine. Intended for user extension — not used within this module itself.

---

## Core Patterns

### Pattern 1 — Inline Vector Construction

Everywhere geometry is assembled inline:
```karel
%from systemlib.klh %import VEC, VEC2D

-- 3D point
origin := VEC(0.0, 0.0, 710.0)

-- 2D polygon vertex (z implicitly 0)
v := VEC2D(edges[3].x, edges[1].y)

-- Fuzzy comparison
IF compare_VEC(measured_pt, expected_pt, 0.01) THEN
    -- points coincide within 10µm
ENDIF
```

---

### Pattern 2 — ZEROPOS for Group-0 Config Extraction

Karel requires a `CONFIG` to construct a XYZWPR with `POS()`. The robot's current zero config is the idiomatic source:
```karel
%include systemvars.klt

VAR ref_pose : XYZWPR

-- Get zero config for group 1, then build pose with it
ref_pose := POS(230.5, -268.4, 259.1, -37.34, 33.83, -23.01, (ZEROPOS(1).config_data))

-- Or initialize reference frame to zero pose
ref_frame := ZEROPOS(1)
```

---

### Pattern 3 — t_DATA_TYPE as Generic Container Value

Used in hash, graph, and iterator modules to carry any data type in the same node:
```karel
%include systemlib.datatypes.klt
%include systemlib.codes.klt

VAR node : t_DATA_TYPE

-- Store an integer
node.int := 42

-- Store a vector
node.vec := VEC(1.0, 2.0, 3.0)

-- Generic comparison (used in graph/KD-tree sorting):
IF system__tdata_glte(node_a, node_b, C_VEC, C_LESSER) THEN
    -- node_a's vector norm is less than node_b's
ENDIF
```

---

### Pattern 4 — Atomic Wrappers for PATH Nodes

Karel PATH requires STRUCTURE node types. `t_INTEGER`, `t_REAL`, etc., enable atomic values in PATH-backed collections:
```karel
%include systemlib.types.klt

-- In queue.klc or iterator.klc — store an integer in a PATH:
VAR node : t_INTEGER
node.v := 99
APPEND_NODE(my_path)
my_path[PATH_LEN(my_path)] := node

-- Retrieve:
result := my_path[1].v
```

---

### Pattern 5 — Leader Frame API for Coordinated Dual-Arm

```karel
%from systemlib.klh %import system__set_leader_frame, system__get_leader_frame, system__mask_leader_frame

VAR frm : XYZWPR

-- Set full frame for pair 1, leader 2
frm := POS(100, 200, 300, 0, 0, 0, (ZEROPOS(1).config_data))
system__set_leader_frame(1, 2, frm)

-- Later: update only the Z axis
system__mask_leader_frame(1, 2, 3, 450.0)   -- axs=3 → Z

-- Read back
frm := system__get_leader_frame(1, 2)
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Accessing `nde.real` on `t_DATA_TYPE` | Compile error (Karel keyword `REAL` conflicts) | Use `nde.rl` — the field is deliberately named `rl` |
| Passing a POSITION (homogeneous) to `system__set_leader_frame` | Type mismatch compile error | The routine takes XYZWPR; convert first if needed |
| Using `C_VEC` comparator expecting component-wise compare | Gets scalar norm comparison, not per-axis | `system__tdata_glte` with `C_VEC` compares Euclidean norms; implement per-axis logic separately |
| Forgetting `%include systemlib.datatypes.klt` before using `t_DATA_TYPE` | TYPE not found / compile error | Include the type header before any `VAR : t_DATA_TYPE` declaration |
| Using `ZEROPOS(g)` as a full pose for motion | Robot moves to literal zero position | `ZEROPOS` is for config extraction only (`ZEROPOS(1).config_data`), not a meaningful motion target |
| Expecting `system__time` to have second-level precision | Seconds are always even (0, 2, 4, ...) | Karel's `GET_TIME` has 2-second resolution; the implementation multiplies the 5-bit field × 2 |

---

## Dependencies

**lib/system depends on:**
- `Strings` — `rstrip`, `lstrip`, `i_to_s`, `bin_to_i` used in `system__date`, `system__time`, `system__pns_to_str`
- `ktransw-macros` — `declare_function`, `funcname` for `module__routine` namespace mangling

**Modules that depend on lib/system (direct):**
`pose`, `draw`, `shapes`, `math`, `matrix`, `csv`, `files`, `registers`, `display`, `multitask`, `TPE`, `iterator`, `queue`, `hash`, `graph`, `layout`, `sensors`, `paths/*`

`systemlib` types and macros are so pervasive that it is effectively a transitive dependency of every Layer 2+ module.

---

## Build / Integration Notes

- **Include path:** Headers live in `include/`. Downstream modules reference them as `systemlib.datatypes.klt`, `systemvars.klt`, etc. — rossum puts the include dir on the preprocessor path.
- **Source:** Only `src/systemlib.kl` contains compiled routines. Everything else is header-only (types, macros, constants).
- **TP Interfaces:** `package.json` registers 5 TP-callable wrappers (`sys_pns2sr`, `sys_setldr`, `sys_getldr`, `sys_mskldr`, `str_set`). These are thin shims compiled alongside `systemlib.kl` via `tp-interfaces` in the rossum manifest.
- **Interface dependencies:** `tp-interfaces` list `TPElib`, `registers`, `pose` — these are needed only for TP wrapper compilation, not the core Karel library.
- **No GPP expansion needed:** This module contains no `.klc` class templates. `systemlib.generics.klt` is a macro-only file used as a helper for user-defined generic comparators; it does not generate standalone code.
