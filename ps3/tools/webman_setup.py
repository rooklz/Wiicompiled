#!/usr/bin/env python3
"""webman_setup.py — read, back up and carefully modify webMAN's settings.

webMAN's configuration page is a single GET form with well over a hundred
controls, and submitting it drops anything not included. Changing one field by
hand-writing a query string is therefore a good way to silently reset somebody
else's console configuration.

This parses the *whole* form, reproduces it faithfully (checkboxes only when
checked, radios only the selected one, selects at their selected option), and
lets exactly one field be overridden. It writes a JSON backup first and diffs
the form after submitting, so any unintended change is visible immediately and
can be reverted.

  ./webman_setup.py <ip> backup <file.json>
  ./webman_setup.py <ip> set <name> <value>      # verifies afterwards
  ./webman_setup.py <ip> enable <name>           # tick a checkbox
  ./webman_setup.py <ip> disable <name>          # untick a checkbox
  ./webman_setup.py <ip> restore <file.json>

`set` deliberately refuses a name that is not currently being submitted, which
for a checkbox means one that is not ticked -- there is no value to change. So
ticking and unticking are separate operations: they add or remove the field
rather than edit it, and they check the box genuinely exists on the page first,
so a typo cannot silently submit a form with a junk field in it.
"""
import html
import json
import re
import sys
import urllib.parse
import urllib.request

INPUT_RE  = re.compile(r"<input\b[^>]*>", re.I)
SELECT_RE = re.compile(r"<select\b[^>]*>(.*?)</select>", re.I | re.S)
OPTION_RE = re.compile(r"<option\b([^>]*)>(.*?)</option>", re.I | re.S)
ATTR_RE   = re.compile(r'(\w[\w-]*)\s*=\s*"([^"]*)"', re.I)


def attrs(tag):
    return {k.lower(): html.unescape(v) for k, v in ATTR_RE.findall(tag)}


def fetch(ip, path="/setup.ps3"):
    with urllib.request.urlopen(f"http://{ip}{path}", timeout=15) as r:
        return r.read().decode("utf-8", "replace")


def parse_form(page):
    """Return [(name, value)] exactly as a browser would submit it."""
    fields = []
    seen_radio = set()

    for tag in INPUT_RE.findall(page):
        a = attrs(tag)
        name = a.get("name")
        if not name:
            continue
        typ = a.get("type", "text").lower()
        val = a.get("value", "")

        if typ == "checkbox":
            # Unchecked boxes are not submitted at all.
            if "checked" in a:
                fields.append((name, val or "1"))
        elif typ == "radio":
            if "checked" in a and name not in seen_radio:
                seen_radio.add(name)
                fields.append((name, val))
        elif typ in ("submit", "button", "reset", "image", "file"):
            continue
        else:
            fields.append((name, val))

    for m in re.finditer(r"<select\b([^>]*)>(.*?)</select>", page, re.I | re.S):
        a = attrs("<select " + m.group(1) + ">")
        name = a.get("name")
        if not name:
            continue
        chosen = None
        first = None
        for oa, _text in OPTION_RE.findall(m.group(2)):
            oattrs = attrs("<option " + oa + ">")
            v = oattrs.get("value", "")
            if first is None:
                first = v
            if "selected" in oattrs:
                chosen = v
                break
        fields.append((name, chosen if chosen is not None else (first or "")))

    return fields


def submit(ip, fields):
    qs = urllib.parse.urlencode(fields)
    url = f"http://{ip}/setup.ps3?{qs}"
    with urllib.request.urlopen(url, timeout=25) as r:
        return r.read().decode("utf-8", "replace")


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    ip, action = sys.argv[1], sys.argv[2]

    page = fetch(ip)
    fields = parse_form(page)
    before = dict(fields)

    if action == "backup":
        path = sys.argv[3]
        with open(path, "w") as f:
            json.dump(fields, f, indent=1)
        print(f"saved {len(fields)} settings to {path}")
        return 0

    if action == "set":
        name, value = sys.argv[3], sys.argv[4]
        if name not in before:
            sys.exit(f"no such setting: {name}")
        print(f"{name}: {before[name]!r} -> {value!r}")
        newfields = [(n, value if n == name else v) for n, v in fields]
        submit(ip, newfields)

        # Verify: re-read and confirm only the intended field moved.
        after = dict(parse_form(fetch(ip)))
        drift = {k: (before[k], after.get(k))
                 for k in before
                 if k != name and after.get(k) != before[k]}
        if after.get(name) != value:
            print(f"  WARNING: {name} reads back as {after.get(name)!r}")
        else:
            print(f"  confirmed: {name} = {after[name]!r}")
        if drift:
            print(f"  WARNING: {len(drift)} other settings changed:")
            for k, (o, n) in list(drift.items())[:10]:
                print(f"    {k}: {o!r} -> {n!r}")
            return 1
        print("  no other settings changed")
        return 0

    if action in ("enable", "disable"):
        name = sys.argv[3]

        # The box has to exist on the page. Without this check a mistyped name
        # would be submitted as an unknown field, which webMAN ignores -- so the
        # operation would report success and do nothing.
        boxes = {attrs(t).get("name") for t in INPUT_RE.findall(page)
                 if attrs(t).get("type", "").lower() == "checkbox"}
        if name not in boxes:
            sys.exit(f"no such checkbox: {name}")

        want = (action == "enable")
        if (name in before) == want:
            print(f"{name} is already {'on' if want else 'off'}")
            return 0

        if want:
            newfields = fields + [(name, "1")]
        else:
            newfields = [(n, v) for n, v in fields if n != name]
        print(f"{name}: {'off -> on' if want else 'on -> off'}")
        submit(ip, newfields)

        after = dict(parse_form(fetch(ip)))
        if (name in after) != want:
            print(f"  WARNING: {name} did not take")
            return 1
        print(f"  confirmed: {name} is {'on' if want else 'off'}")

        drift = {k: (before[k], after.get(k))
                 for k in before
                 if k != name and after.get(k) != before[k]}
        if drift:
            print(f"  WARNING: {len(drift)} other settings changed:")
            for k, (o, n) in list(drift.items())[:10]:
                print(f"    {k}: {o!r} -> {n!r}")
            return 1
        print("  no other settings changed")
        return 0

    if action == "restore":
        with open(sys.argv[3]) as f:
            saved = [tuple(x) for x in json.load(f)]
        submit(ip, saved)
        print(f"restored {len(saved)} settings")
        return 0

    sys.exit(f"unknown action: {action}")


if __name__ == "__main__":
    sys.exit(main())
