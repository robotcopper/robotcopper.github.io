define("discourse/plugins/footnote/initializers/inline-footnotes",["exports","@popperjs/core","discourse/lib/plugin-api","discourse-common/lib/icon-library"],(function(t,e,o,n){"use strict"
let i
function r(t){const o=document.getElementById("footnote-tooltip"),n=o?.dataset.footnoteId,r=t.target,s=r.dataset.footnoteId
if(i?.destroy(),o?.removeAttribute("data-show"),o?.removeAttribute("data-footnote-id"),!t.target.classList.contains("expand-footnote"))return
if(t.preventDefault(),t.stopPropagation(),n===s)return
const d=o.querySelector(".footnote-tooltip-content")
let a=r.closest(".cooked")
null!=a.dataset.refPostId&&(a=document.querySelector(`article[data-post-id="${a.dataset.refPostId}"] .cooked`))
const l=a.querySelector(s)
d.innerHTML=l.innerHTML,o.dataset.show="",o.dataset.footnoteId=s,i?.destroy(),i=(0,e.createPopper)(r,o,{modifiers:[{name:"arrow",options:{element:o.querySelector("#arrow")}},{name:"preventOverflow",options:{altAxis:!0,padding:5}},{name:"offset",options:{offset:[0,12]}}]})}Object.defineProperty(t,"__esModule",{value:!0}),t.default=void 0
t.default={name:"inline-footnotes",initialize(t){t.lookup("service:site-settings").display_footnotes_inline&&(document.body.append(function(){const t=document.createElement("template")
return t.innerHTML='\n    <div id="footnote-tooltip" role="tooltip">\n      <div class="footnote-tooltip-content"></div>\n      <div id="arrow" data-popper-arrow></div>\n    </div>\n  '.trim(),t.content.firstChild}()),window.addEventListener("click",r,!0),(0,o.withPluginApi)("0.8.9",(t=>{t.decorateCookedElement((t=>function(t){const e=t.querySelectorAll("sup.footnote-ref")
e.forEach((t=>{const e=t.querySelector("a")
if(!e)return
const o=document.createElement("a")
o.classList.add("expand-footnote"),o.innerHTML=(0,n.iconHTML)("ellipsis"),o.href="",o.role="button",o.dataset.footnoteId=e.getAttribute("href"),t.after(o)})),e.length&&t.classList.add("inline-footnotes")}(t)),{onlyStream:!0,id:"inline-footnotes"}),t.onPageChange((()=>{i?.destroy()
const t=document.getElementById("footnote-tooltip")
t?.removeAttribute("data-show"),t?.removeAttribute("data-footnote-id")}))})))},teardown(){i?.destroy(),window.removeEventListener("click",r),document.getElementById("footnote-tooltip")?.remove()}}})),define("discourse/plugins/footnote/lib/discourse-markdown/footnotes",["exports"],(function(t){"use strict"
Object.defineProperty(t,"__esModule",{value:!0}),t.setup=function(t){t.registerOptions(((t,e)=>{t.features.footnotes=window.markdownitFootnote&&!!e.enable_markdown_footnotes})),t.allowList(["ol.footnotes-list","hr.footnotes-sep","li.footnote-item","a.footnote-backref","sup.footnote-ref"]),t.allowList({custom(t,e,o){if(("a"===t||"li"===t)&&"id"===e)return!!o.match(/^fn(ref)?\d+$/)}}),window.markdownitFootnote&&t.registerPlugin(window.markdownitFootnote)}}))

//# sourceMappingURL=footnote-6cfefb632787a8ba84a0880c8555b950f4da823ac0e15e8ba6900c8152e6fa77.map
//!

;
