"""Headless browser test of the radar web page (navico.py serve).

Checks that every control shows the value the radar reports, that the scope
draws echoes, that there are no console errors, and (unless --read-only)
drives a few controls through the page, confirms the radar followed, and puts
them back as they were.

Run through tests/run_uitest.sh, which sets up the browser environment.
"""
import argparse
import json
import sys
import time
import urllib.request

from playwright.sync_api import sync_playwright


def api_state(url):
    with urllib.request.urlopen(url + "api/state") as r:
        return json.load(r)["state"]


def expected(state):
    """Control id -> value the page should show, straight from the reports."""
    s, e = state.get("setup", {}), state.get("setup_extended", {})
    cfg, inst = state.get("config", {}), state.get("installation", {})
    exp = {
        "mode": e.get("use_mode") or s.get("use_mode"),
        "doppler": e.get("doppler"),
        "rain": s.get("rain"),
        "gain": int(str(s.get("gain", "0")).split()[0]),
        "interference": s.get("interference_rejection"),
        "target-boost": s.get("target_boost"),
        "target-expansion": s.get("target_expansion"),
        "local-interference": e.get("local_interference_rejection"),
        "noise": e.get("noise_rejection"),
        "target-separation": e.get("target_separation"),
        "scan-speed": e.get("scan_speed"),
        "sea-state": e.get("sea_state"),
        "sea": e.get("sea_clutter"),
        "sidelobe": e.get("sidelobe"),
        "range": s.get("range_m"),
        "doppler-speed": e.get("doppler_speed_m_s"),
        "accent-light": cfg.get("accent_light"),
        "bearing-alignment": cfg.get("bearing_alignment_deg"),
        "parking-angle": cfg.get("parking_angle_deg"),
        "antenna-height": cfg.get("antenna_height_m"),
    }
    if "antenna_forward_mm" in inst:
        exp["antenna-offset"] = (f"{inst['antenna_forward_mm'] / 1000},"
                                 f"{inst['antenna_starboard_mm'] / 1000}")
    return {k: v for k, v in exp.items() if v is not None}


def same(a, b):
    """Compare as numbers where possible ("75" == 75.0, "0,0" == "0.0,0.0")."""
    try:
        return [float(x) for x in str(a).split(",")] == [float(x) for x in str(b).split(",")]
    except ValueError:
        return str(a) == str(b)


def ui_values(page):
    return page.evaluate("""() => {
        const out = {};
        document.querySelectorAll('[id^="ctl-"]').forEach(el => {
            const label = el.tagName === 'SELECT' && el.selectedIndex >= 0
                ? el.options[el.selectedIndex].text : null;
            out[el.id.slice(4)] = {value: el.value, label};
        });
        return out;
    }""")


def compare(page, url, title):
    exp = expected(api_state(url))
    ui = ui_values(page)
    bad = 0
    print(f"\n== {title}")
    for k, want in exp.items():
        got = ui.get(k, {})
        ok = same(got.get("value"), want)
        bad += not ok
        lab = f" ({got['label']})" if got.get("label") else ""
        print(f"  {'OK ' if ok else 'BAD'} {k:20} radar={want!s:12} page={got.get('value')!s}{lab}")
    for k, v in ui.items():
        if k not in exp:
            print(f"  --  {k:20} not reflected from radar (page shows {v['value']!r})")
    return bad


def wait_for(cond, timeout=8):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.3)
    return False


def set_slider(page, name, v):
    page.evaluate("""([n, v]) => { const el = document.getElementById('ctl-' + n);
        el.value = v; el.dispatchEvent(new Event('input')); el.dispatchEvent(new Event('change')); }""",
                  [name, v])


def auto_button(page, name):
    return page.evaluate(
        "n => document.getElementById('ctl-' + n).nextSibling.querySelector('button').className",
        name)


def drive_controls(page, url):
    """Change controls through the page, check the radar, restore. Returns failures."""
    bad = 0
    st = api_state(url)
    orig_rain = st["setup"]["rain"]
    orig_dop = st["setup_extended"]["doppler"]
    orig_gain = str(st["setup"]["gain"])
    print("\n== end-to-end through the page")

    for v in (10 if orig_rain != 10 else 20, orig_rain):
        set_slider(page, "rain", v)
        ok = wait_for(lambda: api_state(url)["setup"]["rain"] == v)
        print(f"  {'OK ' if ok else 'BAD'} slider rain -> {v}: radar rain={api_state(url)['setup']['rain']}")
        bad += not ok

    other = "both" if orig_dop != "both" else "approaching"
    for v in (other, orig_dop):
        page.select_option("#ctl-doppler", v)
        ok = wait_for(lambda: api_state(url)["setup_extended"]["doppler"] == v)
        print(f"  {'OK ' if ok else 'BAD'} select doppler -> {v}: "
              f"radar doppler={api_state(url)['setup_extended']['doppler']}")
        bad += not ok

    if "auto" not in orig_gain:
        page.evaluate("() => document.getElementById('ctl-gain').nextSibling.querySelector('button').click()")
        ok = wait_for(lambda: "auto" in str(api_state(url)["setup"]["gain"])) and \
            wait_for(lambda: auto_button(page, "gain") == "on", 4)
        print(f"  {'OK ' if ok else 'BAD'} gain auto button: radar gain={api_state(url)['setup']['gain']}, "
              f"button class={auto_button(page, 'gain')!r}")
        bad += not ok
        level = int(orig_gain.split()[0])
        set_slider(page, "gain", level)
        ok = wait_for(lambda: str(api_state(url)["setup"]["gain"]) == orig_gain) and \
            wait_for(lambda: auto_button(page, "gain") == "", 4)
        print(f"  {'OK ' if ok else 'BAD'} gain back to manual {level}: "
              f"radar gain={api_state(url)['setup']['gain']}, button class={auto_button(page, 'gain')!r}")
        bad += not ok
    else:
        print("  --  gain auto button skipped (radar gain already on auto)")

    page.wait_for_timeout(2500)
    bad += compare(page, url, "controls vs radar after restore")
    entries = page.evaluate("document.getElementById('log').children.length")
    ok = entries > 0
    print(f"\n  {'OK ' if ok else 'BAD'} command log has {entries} entries")
    bad += not ok
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--url", default="http://127.0.0.1:8090/")
    ap.add_argument("--screenshot", default="ui.png")
    ap.add_argument("--read-only", action="store_true",
                    help="only compare; send nothing to the radar")
    args = ap.parse_args()
    url = args.url if args.url.endswith("/") else args.url + "/"

    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1400, "height": 1000})
        logs = []
        page.on("console", lambda m: logs.append(f"console.{m.type}: {m.text}"))
        page.on("pageerror", lambda e: logs.append(f"pageerror: {e}"))
        page.on("requestfailed", lambda r: logs.append(f"requestfailed: {r.url}"))
        page.goto(url)
        page.wait_for_timeout(5000)

        header = page.evaluate("""() => Object.fromEntries(
            ['status', 'range', 'edge', 'rpm', 'fps', 'heading'].map(
                id => [id, document.getElementById(id).textContent]))""")
        st = api_state(url)
        print("== header", header)
        bad = 0
        ok = header["status"] == st.get("mode", {}).get("status")
        print(f"  {'OK ' if ok else 'BAD'} status matches radar ({st.get('mode', {}).get('status')})")
        bad += not ok

        lit = page.evaluate("""() => {
            const c = document.getElementById('ppi');
            const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
            let n = 0; for (let i = 0; i < d.length; i += 4) if (d[i] + d[i+1] + d[i+2] > 60) n++;
            return n; }""")
        transmitting = header["status"] == "transmit"
        ok = lit > 1000 or not transmitting
        print(f"  {'OK ' if ok else 'BAD'} scope: {lit} lit pixels"
              + ("" if transmitting else " (radar not transmitting)"))
        bad += not ok

        bad += compare(page, url, "controls vs radar")
        page.screenshot(path=args.screenshot, full_page=True)

        if not args.read_only:
            bad += drive_controls(page, url)

        print("\n== console / errors\n" + ("\n".join(logs) or "none"))
        bad += len([entry for entry in logs if "error" in entry.lower()])
        browser.close()
    print(f"\n{'PASS' if bad == 0 else 'FAIL'}: {bad} problems; screenshot {args.screenshot}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
