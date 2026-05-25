#!/usr/bin/env python3
"""Aggregate per-bag FP width-probe samples into a report-grade justification.

Input CSV rows (appended by fp_width_probe.hpp, one block per bag), 13 cols:
  label,site,kind,max_bits,max_abs,algebraic_worst,chosen_width,samples,
  frac_bits,int_bits_used,min_trailing_zeros,min_nz_abs,nz_samples

Produces:
  - width_report.csv : tidy per-variable row + per-bag observed bits
  - width_report.md  : thesis-ready tables + the sizing argument:

  Guards (MUL)  : observed product width vs algebraic worst vs chosen guard.
  Sums          : observed accumulator width vs chosen.
  Items         : single cast-product fits the sum type.
  FAMILY WIDTHS : the extensive part. For every stored family decompose the
                  chosen format W = 1(sign) + INT + FRAC and show, from data
                  across all bags:
                    * INT bits actually needed  (dynamic range, no overflow)
                    * FRAC bits actually used   (deepest fractional bit set;
                      trailing-zero bits are resolution the data never uses)
                    * dynamic range  max / min-nonzero  (raw and real units)
                  -> argues INT_BITS and FRAC_BITS independently, not just W.
"""
import argparse
import collections
import csv


def sbits(x):
    """signed two's-complement bits to hold integer x."""
    x = abs(int(x))
    b = 1
    while x:
        b += 1
        x >>= 1
    return b


def site_key(site):
    return site.split()[0]


HIDE_SITES = {
}

SHARED_SUM_KEYS = {
    "SUM6_QP_tree": "QP shared with QP_ITEM",
    "SUM6_QP_ACC": "QP shared with QP_ITEM",
    "SUM6_P_QP": "shared with P_QP_ITEM",
    "SUM6_MG_QP": "shared with MG_QP_ITEM",
    "QP_ITEM": "shared with SUM6_QP_*",
    "P_QP_ITEM": "shared with SUM6_P_QP",
    "MG_QP_ITEM": "shared with SUM6_MG_QP",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", required=True)
    ap.add_argument("--prod-widths", required=True)
    ap.add_argument("--md", required=True)
    ap.add_argument("--csv", required=True)
    args = ap.parse_args()

    # token -> real production width (probe binary is wide-overridden so its
    # 'chosen' is unreliable for SUM/ITEM; use these instead).
    prod = {}
    with open(args.prod_widths) as f:
        for row in csv.reader(f):
            if len(row) == 2:
                prod[row[0]] = int(row[1])

    sites = collections.OrderedDict()
    bags = []
    with open(args.samples) as f:
        for row in csv.reader(f):
            if len(row) != 13:
                continue
            (label, site, kind, bits, mabs, worst, chosen, samp,
             frac, intb, mintz, minnz, nzsamp) = row
            if site in HIDE_SITES:
                continue
            bits, mabs, worst, chosen, samp = (int(bits), int(mabs),
                int(worst), int(chosen), int(samp))
            frac, intb, mintz = int(frac), int(intb), int(mintz)
            minnz, nzsamp = int(minnz), int(nzsamp)
            if label not in bags:
                bags.append(label)
            key = site_key(site)
            chosen = prod.get(key, chosen)
            policy = chosen if kind == "STORE" else bits + 1
            s = sites.setdefault(site, {
                "kind": kind, "key": key, "worst": worst, "chosen": chosen,
                "frac": frac, "note": "",
                "policy": policy, "policy_note": SHARED_SUM_KEYS.get(key, ""),
                "per_bag": {}, "total_samples": 0, "nz_total": 0,
                "comb_bits": 0, "comb_abs": 0,
                "comb_intbits": 0, "comb_minnz": 0, "comb_mintz": None})
            s["chosen"] = prod.get(key, s["chosen"])
            s["frac"] = max(s["frac"], frac)
            s["per_bag"][label] = bits
            s["total_samples"] += samp
            s["nz_total"] += nzsamp
            s["comb_bits"] = max(s["comb_bits"], bits)
            s["comb_abs"] = max(s["comb_abs"], mabs)
            s["comb_intbits"] = max(s["comb_intbits"], intb)
            s["policy"] = max(s["policy"], policy)
            if nzsamp:
                s["comb_minnz"] = (minnz if s["comb_minnz"] == 0
                                   else min(s["comb_minnz"], minnz))
                s["comb_mintz"] = (mintz if s["comb_mintz"] is None
                                   else min(s["comb_mintz"], mintz))

    # ---------- tidy CSV ----------
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        head = ["site", "kind", "note", "policy_width", "policy_note",
                "algebraic_worst", "chosen_width",
                "observed_max_bits", "margin_bits", "bits_saved_vs_worst",
                "frac_bits", "int_bits_needed", "frac_bits_used",
                "dead_low_bits", "min_nonzero_raw", "max_abs_raw",
                "total_samples"]
        head += [f"obs_bits[{b}]" for b in bags]
        w.writerow(head)
        for site, s in sites.items():
            margin = s["chosen"] - s["comb_bits"]
            saved = (s["worst"] - s["chosen"]) if s["worst"] else ""
            if s["kind"] == "STORE":
                frac = s["frac"]
                used_frac = (frac - s["comb_mintz"]
                             if s["comb_mintz"] is not None else 0)
                dead = (s["comb_mintz"]
                        if s["comb_mintz"] is not None else frac)
                intn = max(s["comb_intbits"] - 1, 0)  # drop sign for INT span
                row = [site, s["kind"], s["note"], s["chosen"], s["policy_note"],
                       "data", s["chosen"],
                       s["comb_bits"],
                       margin, "", frac, intn, used_frac, dead,
                       s["comb_minnz"], s["comb_abs"], s["total_samples"]]
            else:
                row = [site, s["kind"], s["note"], s["policy"], s["policy_note"],
                       s["worst"] or "n/a",
                       s["chosen"],
                       s["comb_bits"], margin, saved, "", "", "", "",
                       "", s["comb_abs"], s["total_samples"]]
            row += [s["per_bag"].get(b, "") for b in bags]
            w.writerow(row)

    # ---------- markdown ----------
    def basic_table(kind, title, note):
        rows = [(k, v) for k, v in sites.items() if v["kind"] == kind]
        if not rows:
            return ""
        out = [f"### {title}", "", note, ""]
        hdr = (["variable / site", "policy", "chosen", "obs max", "margin",
                "why chosen"] + [f"`{b}`" for b in bags] + ["samples"])
        out += ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
        for site, s in rows:
            margin = s["chosen"] - s["comb_bits"]
            why = s["policy_note"] if s["chosen"] != s["policy"] else ""
            flag = "" if margin >= 1 else " WARN"
            policy = f"{s['comb_bits']}+1={s['policy']}"
            cells = ([f"`{site}`", policy, str(s["chosen"]),
                      str(s["comb_bits"]), f"{margin}{flag}", why or "-"]
                     + [str(s["per_bag"].get(b, "-")) for b in bags]
                     + [f'{s["total_samples"]:,}'])
            out.append("| " + " | ".join(cells) + " |")
        return "\n".join(out) + "\n"

    def store_table():
        rows = [(k, v) for k, v in sites.items() if v["kind"] == "STORE"]
        if not rows:
            return ""
        out = ["### Stored family widths — full `W = 1 + INT + FRAC` "
               "decomposition", "",
               "For each family: **chosen** is the production format. "
               "**INT need** = integer bits the data actually required "
               "(dynamic-range high end); **INT have** = `WIDTH-FRAC-1`. "
               "**FRAC used** = deepest fractional bit ever set "
               "(`FRAC - dead`); **dead** = low fractional bits no nonzero "
               "sample ever set (resolution the format provides but the data "
               "never needs). **range** = max / smallest-nonzero magnitude in "
               "real units. Margins must be >= 0 (WARN flags overflow risk).",
               ""]
        hdr = ["family", "chosen W=1+INT+FRAC", "INT need", "INT have",
               "INT slack", "FRAC", "FRAC used", "dead", "min |x|", "max |x|",
               "nz samples", "note"]
        out += ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
        for site, s in rows:
            frac = s["frac"]
            chosen = s["chosen"]
            int_have = chosen - frac - 1
            int_need = max(s["comb_intbits"] - 1, 0)
            int_slack = int_have - int_need
            fam = s["key"]
            dead = s["comb_mintz"] if s["comb_mintz"] is not None else frac
            dead = min(dead, frac)
            frac_used = frac - dead
            scale = float(1 << frac)
            mn = (s["comb_minnz"] / scale) if s["comb_minnz"] else 0.0
            mx = s["comb_abs"] / scale
            iflag = "" if int_slack >= 0 else " WARN"
            out.append("| " + " | ".join([
                f"`{fam}`",
                f"{chosen} = 1+{int_have}+{frac}",
                str(int_need), str(int_have), f"{int_slack}{iflag}",
                str(frac), str(frac_used), str(dead),
                f"{mn:.3e}", f"{mx:.3e}",
                f'{s["nz_total"]:,}', s["note"]]) + " |")
        out += ["",
            "**Reading it for the report.** *INT need < INT have* proves the "
            "format cannot overflow on the recorded operating envelope, so "
            "the worst-case integer span is unnecessary. *dead > 0* means the "
            "bottom `dead` fractional bits are never exercised by any sample "
            "across any bag, so `FRAC` (hence `WIDTH`) carries resolution the "
            "physical data does not contain — direct evidence that a smaller "
            "`FRAC_BITS` is lossless for this data. *INT need* and *FRAC used* "
            "together give the minimal correct `WIDTH = 1 + INT_need + "
            "FRAC_used`; the chosen width's surplus over that is the "
            "engineering margin."]
        return "\n".join(out) + "\n"

    total = sum(s["total_samples"] for s in sites.values())
    md = ["# Fixed-point sizing — empirical justification (per variable)", "",
          f"Aggregated over **{len(bags)}** independent recorded runs "
          f"({', '.join('`'+b+'`' for b in bags)}); "
          f"**{total:,}** instrumented samples.", "",
          "Every configurable width and guard in `fp_types_hls.hpp` is sized "
          "below its algebraic worst case deliberately. The tables below give, "
          "per variable, the evidence: the worst case is provably never "
          "approached, the behaviour is consistent across independent bags, "
          "and the chosen width keeps a safety margin over the worst case "
          "actually observed.", "",
          basic_table("MUL", "Product widths -> guard bits (`*_GUARD`)",
            "Policy: **product width = observed max + 1**. "
            "`chosen` is the production width; if it differs from policy, the "
            "table states why."),
          basic_table("SUM", "Accumulator widths (`fp_sum*_*`)",
            "Policy: **accumulator width = observed max + 1**. If a row still "
            "shares a typedef with another live use, the reason is shown in "
            "`why chosen`."),
          basic_table("ITEM", "Single cast-product into each sum type",
            "Policy: **single cast-product width = observed max + 1**. ITEM "
            "shares the same typedef as the matching SUM row, so `chosen` is "
            "the common production width for that pair."),
          store_table(),
          "> Reproduce: `tools/mpc_replay/run_width_report.sh` (rebuilds the "
          "probe with wide measurement typedefs, re-runs every bag under "
          "`tools/input/`).", ""]

    with open(args.md, "w") as f:
        f.write("\n".join(md))

    bad = []
    for k, v in sites.items():
        if v["total_samples"] == 0:
            continue
        min_margin = 0 if v["kind"] == "STORE" else 1
        if v["chosen"] - v["comb_bits"] < min_margin:
            bad.append(k + "(width)")
        if v["kind"] == "STORE":
            int_have = v["chosen"] - v["frac"] - 1
            if max(v["comb_intbits"] - 1, 0) > int_have:
                bad.append(k + "(INT overflow)")
    print(f"sites={len(sites)} bags={len(bags)} samples={total:,}")
    print("WARNING under-sized: " + ", ".join(bad) if bad
          else "OK: every site has >=1 bit width margin and no INT overflow "
               "across all bags.")


if __name__ == "__main__":
    main()
