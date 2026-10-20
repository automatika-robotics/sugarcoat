function _$ (sel, root = document) {return root.querySelector(sel)}
function _$$(sel, root = document) {return root.querySelectorAll(sel)}
function $H(ss, ...values) {
  const r = document.createElement('div');
  r.innerHTML = ss.reduce((r,s,i) => r+s + (values[i] || ''), '').trim();
  return r.firstElementChild;
}

function $E(t, attrs = {}, children = []) {
  if (typeof t === 'string') {
    const { style, dataset, className, cls, ...rest } = attrs;
    const e = document.createElement(t);
    
    if (style && typeof style === 'object') Object.assign(e.style, style);
    if (className || cls) e.className = className || cls;
    if (dataset) Object.assign(e.dataset, dataset);
    
    Object.entries(rest).forEach(([k, v]) => {
      if (k.startsWith('on') && typeof v === 'function') e.addEventListener(k.slice(2), v);
      else e.setAttribute(k, v);
    });

    const processChild = (child) => {
      if (Array.isArray(child)) return $E(...child);
      else if (typeof child === 'string' || child instanceof Node) return child;
      return '';
    };

    e.append(...(Array.isArray(children) ? children.map(processChild) : [processChild(children)]));
    return e;
  } else return t;
}

function proc_htmx(sel, func) {
  htmx.onLoad(elt => {
    const elements = Array.from(htmx.findAll(elt, sel));
    if (elt.matches(sel)) elements.unshift(elt)
    elements.forEach(func);
  });
}

function domReadyExecute(cb) {
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", cb)
  else cb();
}

async function hx_multipart(path, context = {}, verb = 'POST') {
    const form = $H`<form style="display:none;" hx-encoding="multipart/form-data"></form>`;
    document.body.appendChild(form);
    try { await htmx.ajax(verb, path, { ...context, source: form }) }
    finally { form.remove() }
}
