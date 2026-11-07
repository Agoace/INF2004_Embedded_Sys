// Run the real dashboard page script with a minimal fake DOM.
// Input on stdin: {"page": html, "steps": [...]}. Output: JSON trace on stdout.
// Steps: {"render": snapshot} | {"click": id} | {"respond": {status, body} or "throw"}
//        | {"advance_ms": n}. Only the browser APIs the page uses are faked.
"use strict";
const vm = require("vm");

const canvasCalls = [];
function makeNode(id) {
  return {
    id, hidden: id === "restart" || id === "cmderror" || id === "banner",
    disabled: false, title: "", className: "", textContent: "", dataset: {},
    classList: { toggle() {} },
    children: [],
    appendChild(child) { this.children.push(child); return child; },
    replaceChildren() { this.children = []; },
    insertRow() { return { insertCell: () => makeNode(""), appendChild() {} }; },
    // One child per selector, so a test can read text the page wrote there.
    querySelector(sel) {
      this.found = this.found || {};
      return (this.found[sel] = this.found[sel] || makeNode(""));
    },
    closest() { return this; },
    clientWidth: 300, clientHeight: 120,
    getContext() {
      return new Proxy({}, { get: (t, k) => (k in t ? t[k] : (...args) => {
        if (args.some(v => typeof v === "number" && !Number.isFinite(v)))
          throw new Error("non-finite canvas coordinate in " + k);
        canvasCalls.push([k, ...args]);
      }), set: (t, k, v) => { t[k] = v; return true; } });
    },
    addEventListener(type, fn) { this.listener = fn; },
  };
}

async function main() {
  const input = JSON.parse(require("fs").readFileSync(0, "utf8"));
  const html = input.page;
  const meta = name => {
    const m = html.match(new RegExp('<meta name="' + name + '" content="([^"]*)">'));
    return m ? m[1] : null;
  };
  const script = html.match(/<script nonce="[^"]+">([\s\S]*?)<\/script>/)[1];
  const nodes = {};
  const node = id => (nodes[id] = nodes[id] || makeNode(id));
  for (const name of ["dump", "calibrate", "start", "estop"]) node(name).dataset.command = name;
  const trace = { posts: [], reloads: 0 };
  let now = input.now || 1000000;
  let pending = [];
  const session = Object.assign({}, input.session || {});
  const context = {
    console, JSON, Object, String, Number, Boolean, Math, Promise, Array,
    Date: { now: () => now },
    document: {
      querySelector: sel => {
        const m = sel.match(/meta\[name="([^"]+)"\]/);
        if (m) return { content: meta(m[1]) };
        return null;
      },
      querySelectorAll: () => ["dump", "calibrate", "start", "estop"].map(node),
      getElementById: node,
      createElement: () => makeNode(""),
    },
    window: { EventSource: undefined, devicePixelRatio: 1 },
    sessionStorage: {
      getItem: key => (key in session ? session[key] : null),
      setItem: (key, value) => { session[key] = String(value); },
    },
    location: { reload: () => { trace.reloads += 1; } },
    setInterval: () => 1, clearInterval: () => {}, setTimeout: () => 1,
    fetch: (url, options) => {
      if (url === "/api/state") return new Promise(() => {});
      trace.posts.push({ url, headers: options.headers, body: JSON.parse(options.body) });
      return new Promise((resolve, reject) => pending.push({ resolve, reject }));
    },
  };
  vm.createContext(context);
  vm.runInContext(script, context);
  const settle = () => new Promise(r => setImmediate(r));
  for (const step of input.steps) {
    if (step.render) vm.runInContext("render(" + JSON.stringify(step.render) + ")", context);
    if (step.click) {
      const target = node(step.click);
      node("buttons").listener({ target: { closest: () => (target.disabled ? null : target) } });
    }
    if (step.respond) {
      const waiter = pending.shift();
      if (!waiter) throw new Error("no pending command");
      if (step.respond === "throw") waiter.reject(new TypeError("Failed to fetch"));
      else {
        const { status, body } = step.respond;
        waiter.resolve({ ok: status >= 200 && status < 300, status,
          json: async () => { if (body === null) throw new SyntaxError("bad json"); return body; } });
      }
    }
    if (step.advance_ms) now += step.advance_ms;
    await settle(); await settle();
  }
  const view = id => ({ hidden: nodes[id] ? nodes[id].hidden : true,
                        text: nodes[id] ? nodes[id].textContent : "",
                        disabled: nodes[id] ? nodes[id].disabled : false });
  trace.cmderror = view("cmderror");
  trace.restart = view("restart");
  trace.message = view("message");
  trace.estop = view("estop");
  trace.session = session;
  trace.canvasCalls = canvasCalls;
  trace.values = (input.expressions || []).map(expression => vm.runInContext(expression, context));
  // Optional: {"views": [[id, selector or null], ...]} returns their text.
  trace.views = (input.views || []).map(([id, sel]) => {
    const base = nodes[id];
    const target = base && sel ? (base.found || {})[sel] : base;
    return { id, sel, text: target ? target.textContent : null,
             className: target ? target.className : null };
  });
  process.stdout.write(JSON.stringify(trace));
}
main().catch(error => { process.stderr.write(String(error.stack || error)); process.exit(1); });
