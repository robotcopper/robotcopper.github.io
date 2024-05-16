---
layout: default
permalink: /index.html # allow renaming of the cover page file
title: "<i class='fas fa-home'></i>&nbsp; Home" 
time: 2024-05-12
---
<center>
 <h1 style="font-size: 36px; font-weight: bold;"> About Me </h1>
</center>

<!--
![CI](https://github.com/JV-conseil/jekyll-theme-read-the-docs/workflows/CI/badge.svg?branch=develop)
[![License BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)
-->

> Read the Docs 📖 like Jekyll Theme for GitHub Pages

## What it does?

{: style="margin-left: 100px" }
This theme is inspired by [sphinx-rtd-theme](https://github.com/jekyll-theme-read-the-docs/sphinx_rtd_theme) and refactored with:

{: style="margin-left: 100px" }
- [@primer/css](https://github.com/primer/css)
- [github-pages](https://github.com/github/pages-gem) ([dependency versions](https://pages.github.com/versions/))

## Quick start

```yml
remote_theme: https://github.com/JV-conseil/jekyll-theme-read-the-docs
```

You can [generate][rundocs/starter] with the same files and folders from [rundocs/starter][rundocs/starter]

## Usage

Documentation that can guide how to create with Github pages, please refer to [rundocs/jekyll-rtd-theme](https://github.com/rundocs/jekyll-rtd-theme) for details

## Features

- Shortcodes (Toasts card, mermaid)
- Pages Plugins (emoji, gist, avatar, mentions)
- Auto generate sidebar
- [Attribute List Definitions](https://kramdown.gettalong.org/syntax.html#attribute-list-definitions) (Primer/css utilities, Font Awesome 4)
- Service worker (caches)
- SEO (404, robots.txt, sitemap.xml)
- Canonical Link (Open Graph, Twitter Card, Schema data)

## Options

| name          | default value        | description       |
| ------------- | -------------------- | ----------------- |
| `title`       | repo name            |                   |
| `description` | repo description     |                   |
| `url`         | user domain or cname |                   |
| `baseurl`     | repo name            |                   |
| `lang`        | `en`                 |                   |
| `direction`   | `auto`               | `ltr` or `rtl`    |
| `highlighter` | `rouge`              | Cannot be changed |

```yml
# folders sort
readme_index:
  with_frontmatter: true

meta:
  key1: value1
  key2: value2
  .
  .
  .

google:
  gtag:
  adsense:
  site_verification:

# GDPR compliant alternative to Google Analytics
mouseflow:
  project_api_key:

posthog:
  project_api_key:

telemetry:
  app_id:
  user_identifier:

umami:
  website_id:

mathjax: # this will prased to json, default: {}

# NOTE: mermaid custom link are no longer supported
# instead mermaid is updated to the lastest version
# available through cdn.jsdelivr.net
# mermaid:
#   custom:     # mermaid link
#   initialize: # this will prased to json, default: {}

scss:   # also _includes/extra/styles.scss
script: # also _includes/extra/script.js

translate:
  # shortcodes
  danger:
  note:
  tip:
  warning:
  # 404
  not_found:
  # copyright
  revision:
  # search
  searching:
  search:
  search_docs:
  search_results:
  search_results_found: # the "#" in this translate will replaced with results size!
  search_results_not_found:

plugins:
  - jemoji
  - jekyll-avatar
  - jekyll-mentions
```

## Sponsorship

If this project helps you, you can offer me a cup of coffee ☕️ :-)

<!-- links -->

[rundocs/starter]: https://github.com/rundocs/starter
