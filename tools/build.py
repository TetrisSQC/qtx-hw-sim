"""Copy shared sources into a Quartex project, compile it via the IDE MCP server,
print errors. Usage: python build.py <project-dir> [--run args...]"""
import sys, os, shutil, subprocess, json, glob
here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, "..", "src")
proj = sys.argv[1]
# src/local/ (not checked in: the private ROM unit) overrides the stubs of the same name in src/
files = {os.path.basename(f): f for f in glob.glob(os.path.join(src, "*.pas"))}
files.update({os.path.basename(f): f for f in glob.glob(os.path.join(src, "local", "*.pas"))})
for name, f in files.items():
    dst = os.path.join(proj, name)
    if not os.path.exists(dst) or open(f, "rb").read() != open(dst, "rb").read():
        shutil.copyfile(f, dst)
def qx(tool, args="{}"):
    r = subprocess.run([sys.executable, os.path.join(here, "qx.py"), tool, args], capture_output=True, text=True, encoding="utf-8")
    return r.stdout
info = json.loads(qx("get_project_info"))
want = os.path.normcase(os.path.abspath(proj)).rstrip("/" + chr(92))
have = os.path.normcase(info.get("path", "")).rstrip("/" + chr(92))
if want != have:
    qx("save_all")
    print(qx("open_project", json.dumps({"path": os.path.abspath(proj) + os.sep})).strip()[:200])
    ini = os.path.join(proj, "app.config.ini")
    if os.path.exists(ini):     # the IDE rewrites it: keep the JS readable
        t = open(ini, encoding="utf-8-sig").read()
        if "Obfuscation=1" in t:
            open(ini, "w", encoding="utf-8-sig").write(t.replace("Obfuscation=1", "Obfuscation=0"))
out = json.loads(qx("compile"))
errs = json.loads(qx("get_errors"))
for e in errs.get("errors", []):
    print("%s(%s,%s) %s: %s" % (e["filename"], e["line"], e["column"], e["severity"], e["message"]))
if out.get("errorCount"):
    print("BUILD FAILED, %d errors" % out["errorCount"]); sys.exit(1)
print("build ok (%d warnings)" % out.get("warningCount", 0))
if len(sys.argv) > 2 and sys.argv[2] == "--run":
    sys.exit(subprocess.call(["node", "--stack-size=4000", os.path.join(proj, "index.js")] + sys.argv[3:]))
