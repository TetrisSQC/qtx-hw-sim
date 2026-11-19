"""Copy what goes into the public repository (qtx/ without what .gitignore excludes) to a fresh folder.
Usage: python tools/export_github.py <dest-dir>   (the folder must not exist yet)"""
import os, sys, shutil, fnmatch

root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
dest = sys.argv[1]
if os.path.exists(dest):
    sys.exit(dest + " exists: remove it first (or pick another folder)")
pats = [l.strip() for l in open(os.path.join(root, ".gitignore"), encoding="utf-8")
        if l.strip() and not l.startswith("#")]


def ignored(rel):
    for p in pats:
        if p.endswith("/"):
            if rel.startswith(p) or ("/" + p) in ("/" + rel):
                return True
        elif fnmatch.fnmatch(rel, p) or fnmatch.fnmatch(os.path.basename(rel), p):
            return True
    return False


n = 0
for d, dirs, files in os.walk(root):
    for f in files:
        rel = os.path.relpath(os.path.join(d, f), root).replace(os.sep, "/")
        if ignored(rel):
            continue
        os.makedirs(os.path.dirname(os.path.join(dest, rel)) or dest, exist_ok=True)
        shutil.copy2(os.path.join(root, rel), os.path.join(dest, rel))
        n += 1
print("%d files -> %s" % (n, dest))
