// usage: node cdp.mjs <url> <seconds> <out.png> [js-to-eval-before-shot]
import { spawn } from 'node:child_process';
const [url, secs, out, evalJs] = process.argv.slice(2);
const chrome = spawn('C:/Program Files/Google/Chrome/Application/chrome.exe',
  ['--headless=new', '--remote-debugging-port=9333', '--window-size=1500,900',
   '--user-data-dir=' + process.env.PROF, 'about:blank'], { stdio: 'ignore' });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let tgt;
for (let i = 0; i < 50 && !tgt; i++) {
  await sleep(200);
  try { tgt = (await (await fetch('http://127.0.0.1:9333/json')).json()).find(t => t.type === 'page'); } catch {}
}
const ws = new WebSocket(tgt.webSocketDebuggerUrl);
await new Promise(r => ws.onopen = r);
let id = 0; const pend = {};
ws.onmessage = m => { const d = JSON.parse(m.data);
  if (d.id && pend[d.id]) { pend[d.id](d); delete pend[d.id]; }
  if (d.method === 'Runtime.consoleAPICalled') console.log('console:', d.params.args.map(a => a.value ?? a.description).join(' '));
  if (d.method === 'Runtime.exceptionThrown') console.log('EXCEPTION:', JSON.stringify(d.params.exceptionDetails).slice(0, 600));
};
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend[i] = r; ws.send(JSON.stringify({ id: i, method, params })); });
await send('Runtime.enable');
await send('Page.enable');
await send('Emulation.setDeviceMetricsOverride', { width: 1500, height: 900, deviceScaleFactor: 1, mobile: false });
await send('Page.navigate', { url }); if (process.env.PRE) { await sleep(2000); await send('Runtime.evaluate', { expression: process.env.PRE }); }
await sleep(Number(secs) * 1000);
if (evalJs) { const r = await send('Runtime.evaluate', { expression: evalJs, awaitPromise: true, returnByValue: true }); console.log('eval:', JSON.stringify(r.result?.result?.value ?? r.result)); await sleep(1500); }
const r = await send('Runtime.evaluate', { expression: '[...document.querySelectorAll(".speed,.cpu,.status,.info")].map(e=>e.textContent).join(" || ")', returnByValue: true });
console.log('ui:', r.result.result.value);
const shot = await send('Page.captureScreenshot', { format: 'png' });
(await import('node:fs')).writeFileSync(out, Buffer.from(shot.result.data, 'base64'));
ws.close(); chrome.kill();
process.exit(0);
