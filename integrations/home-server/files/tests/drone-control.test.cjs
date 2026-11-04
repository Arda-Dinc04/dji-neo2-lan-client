// Real Unix HTTP sockets; the PC, Supabase and identities are declared fixtures.
const assert = require("node:assert/strict");
const test = require("node:test");
const http = require("node:http");
const fs = require("node:fs/promises");
const os = require("node:os");
const path = require("node:path");
const { once } = require("node:events");
const { DroneControlGateway } = require("../apps/api/dist/drone/drone-control.gateway.js");
const { DroneControlGuard } = require("../apps/api/dist/drone/drone-control.guard.js");

async function pc(handler, run) {
  const root = await fs.mkdtemp(path.join(os.tmpdir(), "neo2-cloud-test-"));
  const socket = path.join(root, "flight.sock");
  process.env.NEO2_CONTROL_SOCKET = socket;
  process.env.NEO2_CONTROL_ENABLED = "true";
  const requests = [];
  const server = http.createServer(async (request, response) => {
    const chunks = [];
    for await (const chunk of request) chunks.push(chunk);
    const entry = { method: request.method, path: request.url, headers: request.headers,
      body: Buffer.concat(chunks) };
    requests.push(entry);
    if (request.url === "/") {
      response.end('<meta name="neo2-token" content="' + "a".repeat(64) + '">');
    } else await handler(entry, response);
  });
  server.listen(socket); await once(server, "listening");
  try { await run(new DroneControlGateway(), requests); }
  finally { server.closeAllConnections(); server.close(); await fs.rm(root, { recursive: true, force: true }); }
}

test("passes original PC windows and raw mutation bytes without browser credentials", async () => {
  const window = Buffer.from('{"ticket":"signed.original.ticket","mutation_sequence":7,"expires_in_seconds":0.25}');
  const body = Buffer.from('{ "action":"land", "parameters":{}, "ingress":{"ticket":"signed.original.ticket"} }');
  await pc((entry, response) => {
    assert.equal(entry.headers.host, "127.0.0.1:8794");
    assert.equal(entry.headers["x-neo2-token"], "a".repeat(64));
    assert.equal(entry.headers.authorization, undefined);
    assert.equal(entry.headers.origin, undefined);
    response.setHeader("Content-Type", "application/json");
    if (entry.path.endsWith("/window")) response.end(window);
    else { assert.deepEqual(entry.body, body); response.end('{"accepted":true}'); }
  }, async (gateway, requests) => {
    assert.deepEqual((await gateway.forward("GET", "window")).body, window);
    assert.equal((await gateway.forward("POST", "intent", body)).status, 200);
    assert.deepEqual(requests.map(r => r.path), ["/", "/api/remote-flight/window", "/api/remote-flight/intent"]);
  });
});

test("duplicate JSON keys reach the PC unchanged for strict rejection", async () => {
  const body = Buffer.from('{"action":"control","action":"land","parameters":{},"ingress":{}}');
  await pc((entry, response) => {
    assert.deepEqual(entry.body, body); response.writeHead(400); response.end('{"error":"duplicate_json_key"}');
  }, async (gateway, requests) => {
    const reply = await gateway.forward("POST", "intent", body);
    assert.equal(reply.status, 400);
    assert.equal(requests.filter(r => r.method === "POST").length, 1);
  });
});

test("retains frame identity and age headers", async () => {
  await pc((entry, response) => {
    response.setHeader("Content-Type", "image/jpeg");
    response.setHeader("X-Neo2-Connection-Generation", "b".repeat(24));
    response.setHeader("X-Neo2-Frame-Age-Seconds", "0.123456789");
    response.setHeader("X-Neo2-Frame-Id", "same-frame");
    response.end(Buffer.from([255, 216, 255, 217]));
  }, async gateway => {
    const reply = await gateway.forward("GET", "frame");
    assert.equal(reply.headers["x-neo2-frame-age-seconds"], "0.123456789");
    assert.equal(reply.headers["x-neo2-frame-id"], "same-frame");
    assert.deepEqual(reply.body, Buffer.from([255, 216, 255, 217]));
  });
});

test("unknown resources and oversized writes never contact the PC", async () => {
  await pc(() => assert.fail(), async (gateway, requests) => {
    await assert.rejects(gateway.forward("POST", "status", Buffer.from("{}")));
    await assert.rejects(gateway.forward("GET", "intent"));
    await assert.rejects(gateway.forward("POST", "intent", Buffer.alloc(8193)));
    assert.equal(requests.length, 0);
  });
});

test("does not follow redirects or retry a mutation after transport loss", async () => {
  await pc((entry, response) => {
    if (entry.path.endsWith("/status")) { response.writeHead(302, { Location: "https://invalid.test/" }); response.end(); }
    else response.destroy();
  }, async (gateway, requests) => {
    await assert.rejects(gateway.forward("GET", "status"));
    await assert.rejects(gateway.forward("POST", "intent", Buffer.from('{"action":"takeoff"}')));
    assert.equal(requests.filter(r => r.path === "/api/remote-flight/intent").length, 1);
  });
});

test("bounds response size and absolute duration, releasing capacity", async () => {
  let oversized = true;
  await pc((entry, response) => { if (oversized) response.end(Buffer.alloc(256 * 1024 + 1)); }, async gateway => {
    await assert.rejects(gateway.forward("GET", "status"));
    assert.equal(gateway.outstanding, 0);
    oversized = false;
    const started = Date.now();
    await assert.rejects(gateway.forward("GET", "status"));
    assert(Date.now() - started < 2500);
    assert.equal(gateway.outstanding, 0);
  });
});

test("one mutation is allowed, LAND can overtake it, and neither is replayed", async () => {
  let held, observed;
  const arrived = new Promise(resolve => { observed = resolve; });
  await pc((entry, response) => {
    if (JSON.parse(entry.body).action === "control") { held = response; observed(); }
    else response.end('{"accepted":true}');
  }, async (gateway, requests) => {
    const control = gateway.forward("POST", "intent", Buffer.from('{"action":"control"}'));
    await arrived;
    await assert.rejects(gateway.forward("POST", "camera", Buffer.from('{"action":"photo"}')));
    await gateway.forward("POST", "intent", Buffer.from('{"action":"land"}'));
    held.end('{"accepted":true}'); await control;
    assert.equal(requests.filter(r => r.method === "POST").length, 2);
    assert.equal(gateway.mutation, false); assert.equal(gateway.landing, false);
  });
});

test("a PC restart invalidates the token without replaying a POST", async () => {
  let reject = false;
  await pc((entry, response) => { response.writeHead(reject ? 401 : 200); response.end("{}"); }, async (gateway, requests) => {
    await gateway.forward("GET", "status"); reject = true;
    assert.equal((await gateway.forward("POST", "intent", Buffer.from('{"action":"land"}'))).status, 401);
    reject = false; await gateway.forward("GET", "status");
    assert.equal(requests.filter(r => r.path === "/").length, 2);
    assert.equal(requests.filter(r => r.method === "POST").length, 1);
  });
});

function authentication() {
  process.env.HOME_SERVER_ALLOWED_EMAIL = "operator@example.test";
  let mono = 1000, wall = 1000000, calls = 0;
  let lookup = async () => ({ data: { user: { id: "auth-user", email: "operator@example.test" } }, error: null });
  const supabase = { client: {
    auth: { getUser: token => { calls++; return lookup(token); } },
    from: () => ({ select: () => ({ eq: () => ({ maybeSingle: async () => ({
      data: { id: "principal", auth_user_id: "auth-user", email: "operator@example.test", display_name: "Operator" }, error: null
    }) }) }) })
  } };
  const guard = new DroneControlGuard(supabase); guard.clock = () => mono; guard.wall = () => wall;
  const token = (exp = 2000, nonce = 0) => "header." + Buffer.from(JSON.stringify({ exp, nonce })).toString("base64url") + ".fixture";
  const context = value => { const request = { headers: { authorization: "Bearer " + value } };
    return { request, switchToHttp: () => ({ getRequest: () => request }) }; };
  return { guard, token, context, supabase, count: () => calls,
    at: value => { mono = value; }, wall: value => { wall = value; }, lookup: value => { lookup = value; } };
}

test("control authentication reuses only a verified principal and keeps token hashes", async () => {
  const a = authentication(); const token = a.token();
  const one = a.context(token), two = a.context(token);
  assert(await a.guard.canActivate(one)); assert(await a.guard.canActivate(two));
  assert.equal(a.count(), 1); assert.equal(two.request.homeServerUser.role, "admin");
  assert(!JSON.stringify([...a.guard.verified.entries()]).includes(token));
});

test("background validation leaves a fresh command path unblocked and applies revocation", async () => {
  const a = authentication(); const token = a.token(); await a.guard.canActivate(a.context(token));
  let reject;
  a.lookup(() => new Promise((_, fail) => { reject = fail; })); a.at(2100);
  assert(await a.guard.canActivate(a.context(token))); assert.equal(a.count(), 2);
  assert(await a.guard.canActivate(a.context(token)));
  reject(new Error("revoked fixture")); await new Promise(resolve => setImmediate(resolve));
  assert.equal(a.guard.verified.size, 0);
});

test("expired cached authority and expired JWTs never regain validity", async () => {
  const a = authentication(); const token = a.token(); await a.guard.canActivate(a.context(token));
  a.lookup(async () => { throw new Error("revoked fixture"); }); a.at(6001);
  await assert.rejects(a.guard.canActivate(a.context(token)));
  const b = authentication(); const expiring = b.token(1001); await b.guard.canActivate(b.context(expiring));
  b.wall(1001001); await assert.rejects(b.guard.canActivate(b.context(expiring)));
});

test("invalid or unauthorized identities never populate the cache", async () => {
  const a = authentication();
  a.lookup(async () => ({ data: { user: { id: "outsider", email: "other@example.test" } }, error: null }));
  await assert.rejects(a.guard.canActivate(a.context(a.token())));
  assert.equal(a.guard.verified.size, 0);
  await assert.rejects(a.guard.canActivate(a.context("not-a-jwt")));
});

test("authentication has a hard capacity bound and permits existing entries to refresh", async () => {
  const a = authentication();
  for (let i = 0; i < 32; i++) await a.guard.canActivate(a.context(a.token(2000, i)));
  await assert.rejects(a.guard.canActivate(a.context(a.token(2000, 33))));
  a.at(2100); assert(await a.guard.canActivate(a.context(a.token(2000, 0))));
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(a.count(), 33);
  assert.equal(a.guard.verified.size, 32);
});

test("real Nest HTTP enforces the guard and forwards the retained raw JSON", async () => {
  const { Module } = require("@nestjs/common");
  const { NestFactory } = require("@nestjs/core");
  const { DroneControlController } = require("../apps/api/dist/drone/drone-control.controller.js");
  const { SupabaseService } = require("../apps/api/dist/supabase.service.js");
  const authenticationFixture = authentication();
  const original = Buffer.from('{"action":"control","action":"land","parameters":{},"ingress":{}}');
  process.env.WEB_ORIGIN = "https://console.example.test";
  await pc((entry, response) => {
    if (entry.path.endsWith("/window")) { response.end('{"ticket":"unchanged","mutation_sequence":3}'); return; }
    assert.deepEqual(entry.body, original); response.writeHead(400); response.end('{"error":"duplicate_json_key"}');
  }, async (gateway, requests) => {
    class FixtureModule {}
    Module({ controllers: [DroneControlController], providers: [
      { provide: DroneControlGateway, useValue: gateway },
      { provide: SupabaseService, useValue: authenticationFixture.supabase }, DroneControlGuard
    ] })(FixtureModule);
    const app = await NestFactory.create(FixtureModule, { rawBody: true, logger: false, abortOnError: false });
    app.setGlobalPrefix("home-server/api");
    await app.listen(0, "127.0.0.1");
    const url = "http://127.0.0.1:" + app.getHttpServer().address().port + "/home-server/api/drone/control/";
    const headers = { Authorization: "Bearer " + authenticationFixture.token(Math.floor(Date.now()/1000)+3600), "Content-Type": "application/json",
      Origin: "https://console.example.test" };
    try {
      assert.equal((await fetch(url + "window")).status, 401); assert.equal(requests.length, 0);
      const window = await fetch(url + "window", { headers });
      assert.equal(window.headers.get("cache-control"), "no-store");
      assert.equal((await window.json()).ticket, "unchanged");
      const before = requests.length;
      assert.equal((await fetch(url + "intent", { method: "POST", headers: {
        ...headers, Origin: "https://other.invalid" }, body: original })).status, 403);
      assert.equal(requests.length, before);
      const reply = await fetch(url + "intent", { method: "POST", headers, body: original });
      assert.equal(reply.status, 400); assert.equal((await reply.json()).error, "duplicate_json_key");
    } finally { await app.close(); }
  });
});
