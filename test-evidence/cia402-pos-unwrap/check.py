#!/usr/bin/env python3
# Summarize the traces: 0x6064 wraps, pos-fb step at each wrap, target wrap,
# drive following error (0x607A - 0x6064, mod 2^32) while operation enabled.
import csv, ctypes, sys
i32 = lambda v: ctypes.c_int32(v).value
for fn in sys.argv[1:] or ["a_unpatched_csv.csv", "b1_patched_csp.csv", "b2_patched_csp.csv"]:
    r = [{k: (float(v) if k in ("pos_fb", "pos_cmd", "vel_fb") else v) for k, v in x.items()} for x in csv.DictReader(open(fn))]
    en = [x for x in r if int(x["sw"]) & 0x6F == 0x27]
    print(f"== {fn}: {len(r)} samples @ 1 ms, {len(en)} operation enabled, fault bit seen: {any(int(x['sw']) & 8 for x in en)}")
    for a, b in zip(r, r[1:]):
        if abs(int(b["act_raw"]) - int(a["act_raw"])) > 2**31:
            print(f"  t={b['t_ms']} ms 0x6064 {a['act_raw']} -> {b['act_raw']}: pos-fb {a['pos_fb']:.4f} -> {b['pos_fb']:.4f} rev (step {b['pos_fb'] - a['pos_fb']:+.4f})")
        if a["drive_ferr_counts"] != "" and abs(int(b["tgt_raw"]) - int(a["tgt_raw"])) > 2**31:
            print(f"  t={b['t_ms']} ms 0x607A {a['tgt_raw']} -> {b['tgt_raw']}: step mod 2^32 {i32(int(b['tgt_raw']) - int(a['tgt_raw'])):+d} counts")
    if en and en[0]["drive_ferr_counts"] != "":
        fe = max(abs(int(x["drive_ferr_counts"])) for x in en)
        print(f"  max |0x607A - 0x6064| while enabled: {fe} counts ({fe / 2**23:.4f} rev), max |vel_fb| {max(abs(x['vel_fb']) for x in en):.3f} rev/s")
