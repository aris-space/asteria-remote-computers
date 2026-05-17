#import "base.typ": page-layout 
#show: page-layout.with(subteam: "SFW", title: "Config Validation File")
#show "TODO": highlight
#show "FIXME": highlight

#let data = json(sys.inputs.metadata-file)
#let bin = data.binary
#let cfg = data.config

#show table: set table(stroke: 0.5pt)

#let new-line(line) = {
  box(line, radius: 30%, outset: (left: 2em, rest: 3pt), fill: green.lighten(80%))
}
#let deleted-line(line) = {
  box(line, radius: 30%, outset: (left: 2em, rest: 3pt), fill: red.lighten(80%))
}

// from <https://forum.typst.app/t/how-to-add-formatting-rules-to-an-already-formatted-raw-block/1853/6>
#show raw.where(lang: "toml", block: true): it => {
  set text(1em / 0.8)
  let (text, lines, ..fields) = it.fields()

  let replacements = ()
  let new-text = ""
  let last-match-end = 0
  for (index, match) in text.matches(regex("(.*) \# ([\+\-])")).enumerate() {
    let unmatched = text.slice(last-match-end, match.start)
    let placeholder = "PLACEHOLDER" + str(index)

    let replacement = match.captures.at(0)
    let diff-mode = match.captures.at(1)
    replacement = raw(lang: it.lang, replacement)
    replacement = if diff-mode == "+" {
      new-line(replacement)
    } else {
      deleted-line(replacement)
    }
    replacement = std.text(1.25em, replacement)

    new-text += unmatched + placeholder
    replacements.push(replacement)
    last-match-end = match.end
  }
  new-text += text.slice(last-match-end)

  if replacements.len() == 0 {
    return it
  }

  show regex("PLACEHOLDER\d+"): it => {
    let index = int(it.text.slice(11))
    replacements.at(index)
  }
  raw(new-text, ..fields)
}

= Configuration Validation File

== Flight Computer

// #raw(bin.name)

#table(
  columns: (1fr, 1fr, 4fr),
  [git commit], [branch], [latest change],
  highlight(raw(bin.commit), fill: orange.lighten(70%)),
  raw(bin.branch),
  [#bin.last_change],
)

== GNC

#let gnc = data.gnc

#table(
  columns: (1fr, 1fr, 4fr),
  [git commit], [branch], [latest change],
  highlight(raw(gnc.commit), fill: orange.lighten(70%)),
  raw(gnc.branch),
  [#gnc.last_change],
)

== Config

#raw(cfg.path)

#table(
  columns: (1fr, 1fr, 4fr),
  [config hash], [branch], [latest change],
  highlight(raw(cfg.hash)),
  raw(cfg.branch),
  [#cfg.last_change],
)

=== Full config
(diff against #raw(cfg.base)):

#raw(data.toml_annotated, lang: "toml", block: true)
