---
layout: default
title: Trigger effects
description: Ready-to-import DualSense adaptive-trigger effects for DS5Dongle-Studio — walls, bows, resistance and vibration captured from real games.
---

# Trigger effects

Adaptive-trigger effects you can drop straight onto a trigger. Download one, open
the [config portal]({{ '/ds5-config-portal.html' | relative_url }}), and use **Load
custom effect** on L2 or R2. The builder's diagram shows exactly how it plays
across the pull; set the condition (usually *While held*) and Save to a slot.

These are captured from games that send them, so you can replay a game's trigger
feel in a game that has none.

{% assign effects = site.data.effects | sort_natural: 'name' %}
<table>
  <thead>
    <tr><th align="left">Effect</th><th align="left">Type</th><th>Trigger</th><th>File</th></tr>
  </thead>
  <tbody>
  {%- for e in effects %}
    <tr>
      <td><strong>{{ e.name }}</strong><br><small>{{ e.desc }}</small></td>
      <td>{{ e.type }}</td>
      <td align="center">{{ e.trigger }}</td>
      <td><a href="{{ e.file | relative_url }}">JSON</a></td>
    </tr>
  {%- endfor %}
  </tbody>
</table>

**Add one:** capture or build it in the portal, click **Export effect** (or the
builder's **Save**), drop the JSON in `shared/effects/`, and append a stanza to
`_data/effects.yml`.

[← Back to home]({{ '/' | relative_url }}) · [Shared profiles]({{ '/profiles.html' | relative_url }})
