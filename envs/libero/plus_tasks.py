"""LIBERO-Plus task selection glue: perturbation-dim classification + mutually
exclusive collect/eval splits.

LIBERO-Plus packs perturbed variants as EXTRA TASKS inside the same suites
(libero_spatial grows 10 -> 2402 tasks); the perturbation dimension is encoded
in the task-name suffix:

    _light_N       lighting            _language_N   instruction rewording
    _add_N         extra objects       _table_N      table texture
    _tb_N          table+background    _view_N       camera viewpoint
    _noise_N       sensor noise        _levelK_sampleM  new-object levels
    (no suffix)    original task

REPRODUCIBILITY HARD RULE: LIBERO-Plus builds task_order_index 1..19 with an
UNSEEDED shuffle at import time — orders differ across processes. Everything
here (and every client) must use task_order_index=0 (identity order); task_id
then indexes libero_task_map order deterministically.

Split rule (mechanical, no hand-picking): within each
perturbation dim, variants are grouped by base task and sorted by variant
number; EVEN positions -> collect subset, ODD -> eval subset. Both subsets
cover every base task with interleaved variant numbers, and are mutually
exclusive by construction. No randomness anywhere.

CLI (run in the pixi sim env):
  python -m envs.libero.plus_tasks --suite libero_spatial --dim add
  python -m envs.libero.plus_tasks --suite libero_spatial --dim add --split eval
"""
import argparse
import re

DIMS = ("noise", "light", "language", "add", "table", "tb", "view")
_LEVEL_RE = re.compile(r"_level\d+_sample\d+")


def classify(task_name):
    """Task name -> perturbation dim tag (see module docstring)."""
    for t in DIMS:
        if f"_{t}_" in task_name:
            return t
    if _LEVEL_RE.search(task_name):
        return "newobj"
    return "original"


def base_task(task_name):
    """Strip the perturbation suffix -> the original task name."""
    for t in DIMS:
        m = re.search(rf"_{t}_\d+", task_name)
        if m:
            return task_name[: m.start()]
    m = _LEVEL_RE.search(task_name)
    return task_name[: m.start()] if m else task_name


def variant_num(task_name):
    m = re.search(r"_(\d+)$", task_name) or re.search(r"_sample(\d+)", task_name)
    return int(m.group(1)) if m else -1


def suite_tasks(suite_name):
    """[(task_id, name, dim)] for a suite, task_order_index=0 (identity).
    Import is deferred: this module stays importable outside the sim env."""
    from libero.libero import benchmark  # noqa: PLC0415
    s = benchmark.get_benchmark_dict()[suite_name](task_order_index=0)
    return [(i, n, classify(n)) for i, n in enumerate(s.get_task_names())]


# ---- official taxonomy (task_classification.json, shipped with LIBERO-plus) ----
# Verified 2026-08-24: json entries' `id - 1` == benchmark task_id (0 mismatches
# over 2402 spatial tasks, name-by-name). Official categories separate Camera
# Viewpoints from Robot Initial States (both hidden in "_view_" filenames) and
# carry difficulty_level 1-5 — always prefer this over classify()'s regex.

CATEGORIES = ("Camera Viewpoints", "Robot Initial States", "Objects Layout",
              "Sensor Noise", "Light Conditions", "Background Textures",
              "Language Instructions")


def official_tasks(suite_name, category=None, levels=None):
    """[(task_id0, name, category, level)] from the official classification.
    category: exact string from CATEGORIES; levels: iterable of ints 1-5."""
    import json  # noqa: PLC0415
    import os  # noqa: PLC0415
    import libero.libero.benchmark as B  # noqa: PLC0415
    path = os.path.join(os.path.dirname(B.__file__), "task_classification.json")
    rows = json.load(open(path))[suite_name]
    out = []
    for t in rows:
        if category is not None and t["category"] != category:
            continue
        if levels is not None and t["difficulty_level"] not in set(levels):
            continue
        out.append((t["id"] - 1, t["name"], t["category"], t["difficulty_level"]))
    return out


def official_split(rows):
    """Mutually exclusive collect/eval split over official rows: group by base
    task, sort by (base, level, id), alternate even->collect / odd->eval."""
    rows = sorted(rows, key=lambda r: (base_task(r[1]), r[3], r[0]))
    collect, ev, prev, k = [], [], None, 0
    for r in rows:
        b = base_task(r[1])
        if b != prev:
            prev, k = b, 0
        (collect if k % 2 == 0 else ev).append(r)
        k += 1
    return collect, ev


def stratified_pick(rows, per_level=4, levels=(1, 2, 3, 4, 5)):
    """Pick up to per_level tasks per difficulty level, spreading across base
    tasks (round-robin) — deterministic, no randomness."""
    pick = []
    for lv in levels:
        cand = [r for r in rows if r[3] == lv]
        cand.sort(key=lambda r: (base_task(r[1]), r[0]))
        seen, chosen, rest = set(), [], []
        for r in cand:
            b = base_task(r[1])
            (chosen if b not in seen else rest).append(r)
            seen.add(b)
        pick += (chosen + rest)[:per_level]
    return pick


def dim_tasks(suite_name, dim):
    """Tasks of one perturbation dim, grouped by base task, variant-sorted."""
    rows = [(i, n) for i, n, d in suite_tasks(suite_name) if d == dim]
    rows.sort(key=lambda r: (base_task(r[1]), variant_num(r[1]), r[0]))
    return rows


def split_collect_eval(rows):
    """Deterministic mutually exclusive split (even -> collect, odd -> eval),
    interleaved within each base task so both subsets cover all base tasks."""
    collect, ev, k, prev = [], [], 0, None
    for i, n in rows:
        b = base_task(n)
        if b != prev:
            prev, k = b, 0
        (collect if k % 2 == 0 else ev).append((i, n))
        k += 1
    return collect, ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_spatial")
    ap.add_argument("--dim", default=None,
                    help=f"one of {DIMS + ('newobj', 'original')}; None = census only")
    ap.add_argument("--split", choices=["collect", "eval"], default=None,
                    help="print only this half of the split")
    ap.add_argument("--ids-only", action="store_true", help="comma list for scripts")
    args = ap.parse_args()

    rows_all = suite_tasks(args.suite)
    if args.dim is None:
        from collections import Counter  # noqa: PLC0415
        c = Counter(d for _, _, d in rows_all)
        print(f"{args.suite}: {len(rows_all)} tasks")
        for k, v in c.most_common():
            print(f"  {v:5d}  {k}")
        return
    rows = dim_tasks(args.suite, args.dim)
    collect, ev = split_collect_eval(rows)
    pick = {"collect": collect, "eval": ev}.get(args.split)
    if args.ids_only:
        for name, part in (("collect", collect), ("eval", ev)):
            if pick is None or part is pick:
                print(f"{name}: " + ",".join(str(i) for i, _ in part))
        return
    print(f"{args.suite} / dim={args.dim}: {len(rows)} tasks "
          f"-> collect {len(collect)} | eval {len(ev)} (互斥,偶/奇交错)")
    for name, part in (("COLLECT", collect), ("EVAL", ev)):
        if pick is None or part is pick:
            print(f"-- {name} --")
            for i, n in part:
                print(f"  {i:5d}  {n}")


if __name__ == "__main__":
    main()
