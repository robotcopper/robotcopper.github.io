define("discourse/plugins/checklist/discourse/initializers/checklist",["exports","discourse/lib/ajax","discourse/lib/ajax-error","discourse/lib/plugin-api","discourse-common/lib/icon-library","discourse-i18n"],(function(e,n,t,c,s,i){"use strict"
function a(e){return 3===e.nodeType&&e.nodeValue.match(/^\s*$/)}function r(e){e.forEach((e=>{let n=e.parentElement
"P"===n.nodeName&&n.parentElement.firstElementChild===n&&(n=n.parentElement),"LI"!==n.nodeName||"UL"!==n.parentElement.nodeName||function(e){let n=e.previousSibling
for(;n;){if(!a(n))return!0
n=n.previousSibling}return!1}(e)||(n.classList.add("has-checkbox"),e.classList.add("list-item-checkbox"),e.nextSibling||e.insertAdjacentHTML("afterend","&#8203;"))}))}function o(e,c){const a=[...e.getElementsByClassName("chcklst-box")]
if(r(a),!c)return
const o=c.widget,l=c.getModel()
l.can_edit&&a.forEach(((e,c)=>{e.onclick=async e=>{const r=e.currentTarget,d=r.classList
if(d.contains("permanent")||d.contains("readonly"))return
const u=d.contains("checked")?"[ ]":"[x]",f=document.createElement("template")
f.innerHTML=(0,s.iconHTML)("spinner",{class:"fa-spin list-item-checkbox"}),r.insertAdjacentElement("afterend",f.content.firstChild),r.classList.add("hidden"),a.forEach((e=>e.classList.add("readonly")))
try{const e=await(0,n.ajax)(`/posts/${l.id}`),t=[];[/`[^`\n]*\n?[^`\n]*`/gm,/^```[^]*?^```/gm,/\[code\][^]*?\[\/code\]/gm,/_(?=\S).*?\S_/gm,/~~(?=\S).*?\S~~/gm].forEach((n=>{let c
for(;null!=(c=n.exec(e.raw));)t.push([c.index,c.index+c[0].length])})),[/([^\[\n]|^)\*\S.+?\S\*(?=[^\]\n]|$)/gm].forEach((n=>{let c
for(;null!=(c=n.exec(e.raw));)t.push([c.index+1,c.index+c[0].length])}))
let s=-1,a=!1
const r=e.raw.replace(/\[( |x)?\]/gi,((n,i,r)=>a||r>0&&"!"===e.raw[r-1]?n:(s+=t.every((e=>e[0]>=r+n.length||r>e[1])),s===c?(a=!0,u):n)))
await l.save({raw:r,edit_reason:(0,i.i18n)("checklist.edit_reason")}),o.attrs.isSaving=!1,o.scheduleRerender()}catch(h){(0,t.popupAjaxError)(h)}finally{a.forEach((e=>e.classList.remove("readonly"))),r.classList.remove("hidden"),r.parentElement.querySelector(".fa-spin").remove()}}}))}Object.defineProperty(e,"__esModule",{value:!0}),e.checklistSyntax=o,e.default=void 0
e.default={name:"checklist",initialize(){(0,c.withPluginApi)("0.1",(e=>function(e){e.container.lookup("service:site-settings").checklist_enabled&&e.decorateCookedElement(o)}(e)))}}})),define("discourse/plugins/checklist/lib/discourse-markdown/checklist",["exports"],(function(e){"use strict"
Object.defineProperty(e,"__esModule",{value:!0}),e.setup=function(e){e.registerOptions(((e,n)=>{e.features.checklist=!!n.checklist_enabled})),e.allowList(["span.chcklst-stroked","span.chcklst-box fa fa-square-o fa-fw","span.chcklst-box checked fa fa-square-check-o fa-fw","span.chcklst-box checked permanent fa fa-square-check fa-fw"]),e.registerPlugin((e=>e.core.ruler.push("checklist",s)))}
const n=/\[( |x)?\]/gi
function t(e,n,t,c){const s=function(e){switch(e){case"x":return"checked fa fa-square-check-o fa-fw"
case"X":return"checked permanent fa fa-square-check fa-fw"
default:return"fa fa-square-o fa-fw"}}(t[1]),i=new c.Token("check_open","span",1)
i.attrs=[["class",`chcklst-box ${s}`]],e.push(i)
const a=new c.Token("check_close","span",-1)
e.push(a)}function c(e,c){let s,i=null,a=0
for(;s=n.exec(e);){if(s.index>a){i=i||[]
const n=new c.Token("text","",0)
n.content=e.slice(a,s.index),i.push(n)}a=s.index+s[0].length,i=i||[],t(i,0,s,c)}if(i&&a<e.length){const n=new c.Token("text","",0)
n.content=e.slice(a),i.push(n)}return i}function s(e){let n,t,s,i,a,r=e.tokens,o=0
for(t=0,s=r.length;t<s;t++)if("inline"===r[t].type)for(i=r[t].children,n=i.length-1;n>=0;n--)if(a=i[n],o+=a.nesting,"text"===a.type&&0===o){const s=c(a.content,e)
s&&(r[t].children=i=e.md.utils.arrayReplaceAt(i,n,s))}}}))

//# sourceMappingURL=checklist-cb8f5f7244720471cd1dd1005c6e04a7d4f96d646997d9b214b6ed4778854dac.map
//!

;
