---
layout: default
title: Shared game profiles
description: Ready-to-import DualSense profiles for DS5Dongle-Studio, one per game — load them in the browser config portal.
---

# Shared game profiles

Ready-to-import DualSense profiles, one per game. Download a profile, open the
[config portal]({{ '/ds5-config-portal.html' | relative_url }}), use **Import
profile**, then **Save to Device** (or into a slot). Profiles that turn Wake on
re-enumerate USB, so import those into a **slot** rather than applying live.

Where a description mentions `--map front` / `rear` / `duplicate`, that's a launch
argument for the audio bridge (set per game in `profile-overrides.txt`), not a
dongle setting — the profile covers the on-dongle half.

{% assign profiles = site.data.profiles | sort_natural: 'game' %}
<table>
  <thead>
    <tr><th align="left">Game</th><th align="left">What it does</th><th>Wake</th><th>Profile</th></tr>
  </thead>
  <tbody>
  {%- for p in profiles %}
    <tr>
      <td><strong>{{ p.game }}</strong></td>
      <td>{{ p.desc }}</td>
      <td align="center">{% if p.wake %}on{% else %}off{% endif %}</td>
      <td><a href="{{ p.file | relative_url }}">JSON</a></td>
    </tr>
  {%- endfor %}
  </tbody>
</table>

**Add yours:** configure the game in the portal, click **Export profile**, drop the
JSON in `shared/profiles/`, and append a stanza to `_data/profiles.yml`. The list
re-sorts itself.

[← Back to home]({{ '/' | relative_url }}) · [Trigger effects]({{ '/effects.html' | relative_url }})
