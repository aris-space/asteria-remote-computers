// This document provides the page layouting and templating for precedures

#let small-text-size = 9pt
#let normal-text-size = 13pt
#let stroke-width = 0.7pt
#let dark-blue = rgb("003366")

// Provides a tag box for use in the header that can be colored depending on the text.
#let title-box(coloring: none, text) = {
  set box(fill: coloring(text)) if coloring != none
  box(inset: 5pt, stroke: stroke-width + black, radius: 2pt)[#text]
}

// Returns the color that the given stage's tag should be.
#let stage-color(stage) = {
  if stage == "Prep" { 
    gray
  } else if stage == "ASM" {
    green
  } else if stage == "OP" {
    yellow
  } else if stage == "CP" {
    red
  }
}

// Returns the color that the given subteams's tag should be.
#let subteam-color(subteam) = {
  if subteam == "AV" or subteam == "AVN" { 
    rgb("FFE6CC")
  } else if subteam == "SFW" {
    rgb("FFF2CC")
  } else if subteam == "STR" {
    rgb("EDEDED")
  } else if subteam == "FDY" {
    rgb("D5E8D4")
  } else if subteam == "PSS" {
    rgb("E1D5E7")
  } else if subteam == "ENG" {
    rgb("F8CECC")
  } else if subteam == "REC" {
    rgb("DAE8FC")
  } else if subteam == "ETB" {
    rgb("F5F5F5")
  } else if subteam == "DACS" {
    rgb("B1E3E6")
  } else if subteam == "SE" {
    rgb("C1CBBC")
  }
}

// adapted from <https://stackoverflow.com/a/78289423>
// get the current header and returns a title box for it
#let current-section-box(level: 1) = {
  let next-headers = query(selector(heading.where(level: level)).after(here()))
  if next-headers.len() != 0 and next-headers.first().location().page() == here().page() {
    return title-box(next-headers.first().body)
  }
  
  let prev-headers = query(selector(heading.where(level: level)).before(here()))
  if prev-headers.len() != 0 {
    return title-box(prev-headers.last().body)
  }
  return ""
}

// Layouts the page to the right geometry and adds the header and footer
#let page-layout(
  title: "TITLE",
  export: "EXPORT",
  subteam: none,
  body,
) = {
  let tag-row = [
    #title-box(title)
    #if subteam != none { title-box(coloring: subteam-color, subteam) }
    // #context{ current-section-box(level: 2) }
  ]
  
  let header = [
    #line(start: (0pt, 0cm), length: 100%, stroke: stroke-width)
    #place(
      left,
      dy: -2.5cm,
      image("assets/ASTERIA_HQ.svg", width: 13%)
    )
    #place(
      center,
      dy: -1cm,
      tag-row
    )
    #place(
      right,
      dy: -2cm,
      image("assets/ARIS_Logo_Black.png", width: 13%)
    )
  ]

  let page-number() = (
    context {
      counter(page).display("1/1", both: true)
    }
  )
  
  let footer = [
    #set text(size: small-text-size)
    #line(start: (0pt, -0pt), length: 100%, stroke: stroke-width)
    #place(
      left,
      dy: 4pt,
      [compiled #datetime.today().display()],
    )
    // #place(
    //   center,
    //   dy: 4pt,
    //   [export for #export],
    // )
    #place(
      right,
      dy: 4pt,
      [#page-number()],
    )
  ]
  
  set page(
    "a4",
    margin: (rest: 1.5cm, bottom: 1.5cm, top: 4cm),
    header: header,
    footer: footer,
  )

  set text(size: normal-text-size)

  body
}
