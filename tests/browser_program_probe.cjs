// Chromium CDP against the explicit UDP-loopback fixture only. No packages.
const fs = require('node:fs'), assert = require('node:assert/strict');
const [port, url, output] = process.argv.slice(2);
assert.match(url, /^http:\/\/127\.0\.0\.1:\d+$/);
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const timeout = setTimeout(() => { console.error('browser program probe timed out'); process.exit(1); }, 25000);
let socket;
(async () => {
  const targets = await (await fetch(`http://127.0.0.1:${Number(port)}/json/list`)).json();
  socket = new WebSocket(targets.find(target => target.type === 'page').webSocketDebuggerUrl);
  await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
  let sequence = 0;
  const pending = new Map(), exceptions = [];
  socket.onmessage = event => {
    const value = JSON.parse(event.data);
    if (value.method === 'Runtime.exceptionThrown') exceptions.push(value.params.exceptionDetails);
    const callback = pending.get(value.id);
    if (callback) { pending.delete(value.id); value.error ? callback.reject(value.error) : callback.resolve(value.result); }
  };
  const command = (method, params = {}) => new Promise((resolve, reject) => {
    const id = ++sequence; pending.set(id, { resolve, reject }); socket.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async expression => {
    const result = await command('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true });
    if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
  };
  const until = async expression => {
    const deadline = Date.now() + 6000;
    while (Date.now() < deadline) { if (await evaluate(expression)) return; await sleep(40); }
    throw new Error('browser condition missing: ' + expression + '\n' + await evaluate('document.body.innerText'));
  };
  await command('Page.enable'); await command('Runtime.enable');
  await command('Emulation.setDeviceMetricsOverride', { width: 1440, height: 1550, deviceScaleFactor: 1, mobile: false });
  await command('Emulation.setFocusEmulationEnabled', { enabled: true });
  await command('Page.navigate', { url });
  await until('document.getElementById("ground") && !document.getElementById("ground").disabled');
  const program = { version: 1, finish: 'land', steps: [
    { kind: 'translate', seconds: .3, right: 50, forward: 0, up: 0 },
    { kind: 'rotate', degrees: 90, channel: 150, timeout_seconds: 3 },
    { kind: 'translate', seconds: .3, right: 0, forward: 50, up: 0 }
  ] };
  await evaluate(`document.getElementById('program-json').value=${JSON.stringify(JSON.stringify(program, null, 2))};document.getElementById('program-json').dispatchEvent(new Event('input'))`);
  assert(await evaluate('document.getElementById("program-start").disabled'));
  await evaluate('document.getElementById("ground").onclick()');
  await until('!document.getElementById("arm").disabled');
  await evaluate('document.getElementById("arm").onclick()');
  await command('Input.dispatchKeyEvent', { type: 'keyDown', code: 'Space', key: ' ', windowsVirtualKeyCode: 32 });
  await until('!document.getElementById("takeoff").disabled');
  await evaluate('document.getElementById("takeoff").onclick()');
  await until('!document.getElementById("confirm-takeoff").disabled');
  await evaluate('document.getElementById("confirm-takeoff").onclick()');
  await until('!document.getElementById("program-start").disabled');
  await evaluate('document.getElementById("program-start").onclick()');
  await until('document.getElementById("program-state").innerText.includes("Mouvements terminés")');
  const summary = await evaluate(`({message:document.getElementById('program-state').innerText,
    completed:document.getElementById('program-progress').value,
    maximum:document.getElementById('program-progress').max,
    profile:document.getElementById('image-state').innerText})`);
  assert.equal(summary.completed, 3); assert.equal(summary.maximum, 3);
  assert.match(summary.profile, /SIMULATION/);
  const image = await command('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true });
  fs.writeFileSync(output, Buffer.from(image.data, 'base64'));
  assert.deepEqual(exceptions, []); console.log(JSON.stringify(summary));
})().catch(error => { console.error(error); process.exitCode = 1; }).finally(() => {
  clearTimeout(timeout); socket?.close();
});
